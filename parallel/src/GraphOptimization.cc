#include "GraphOptimization.h"

GraphOptimizer::GraphOptimizer(const ParameterReader& para, bool verbose): paraReader(para) {   

    //initialize the optimizer
    linearSolver = new SlamLinearSolver();
    solver_ptr = new SlamBlockSolver(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    solver->setUserLambdaInit(1e-10);
    optimizer.setVerbose(verbose);
    n_vertices = 0;

    //initializing the detector
    Params params;
    params.image_rows = para.getData<int>("IMAGE_W");
    params.image_cols = para.getData<int>("IMAGE_H");
    params.use_nss = true; // use normalized similarity score instead of raw score
    params.alpha = 0.5; // nss threshold
    params.k = 1; // a loop must be consistent with 1 previous matches
    params.geom_check = GEOM_DI; // use direct index for geometrical checking
    params.di_levels = 2; // use two direct index levels

    //load the voc files
    string m_vocfile = para.getData<string>("m_vocfile");
    voc.reset(new OrbVocabulary());

    cout << "Loading " <<m_vocfile << " vocabulary..." << endl;
    bool bVocLoad = voc->loadFromTextFile(m_vocfile);
    if (!bVocLoad) {
        cerr << "Loading error!" << endl;
        exit(1);
    }

  //keyframe selection initialized
   cout << "Key frame initialization started" << endl;
   keyFrameSelection.reset(new KeyFrameSelection(*voc, 0.8));
   cout << "Key frame initialization finishes!" << endl;

  //loop detector initialized
   loopDetector.reset(new OrbLoopDetector(*voc, params));
   //loopDetector->allocate(num);

   //start the main thread;
   opthread = make_shared<thread>( bind( &GraphOptimizer::mainLoop , this ) );

}

void GraphOptimizer::insertFrame(Frame::Ptr frame) {

    if (keyFrames.size() == 0) {
        unique_lock<mutex> lck(keyframes_mutex);
        keyFrames.push_back(frame);
        addVertex(frame->id, frame->T_f_w);
        ref_frame = frame;
        return;
    } 

    //keyframe processing
    // DetectionResult result;

    if(!keyFrameSelection->process(frame->descriptors)) return;

    //loop detection
    DetectionResult result;
    loopDetector->detectLoop(frame->KeyPoints, frame->descriptors, result);
    detect_sequence[detect_start++] = frame->id;

    if(result.detection() && (result.query - result.match > 100)) {
         d_count++;
         if (d_count > 5) loopfound = true;
            matchingIdx = detect_sequence[result.match];
            cout << "- Loop found with image " << matchingIdx << "!"<< endl;
    } else {
        d_count = 0;
        loopfound = false;
        cout << "No loop detected " << endl;
    }

    //find process 
    Eigen::Isometry3d delta = ref_frame->getTransform().inverse() * frame->getTransform();
    unique_lock<mutex> lck(keyframes_mutex);
    keyFrames.push_back( frame );

    if (loopfound) {
         addVertex(frame->id, frame->T_f_w, matchingIdx);
         Eigen::Matrix<double,6,6>  information = Eigen::Matrix<double,6,6>::Identity();
         addEdge(ref_frame->id, matchingIdx, delta, information);

    } else {
        addVertex(frame->id, frame->T_f_w);
        Eigen::Matrix<double,6,6>  information = Eigen::Matrix<double,6,6>::Identity();
        addEdge(ref_frame->id, frame->id, delta, information);
    }
    ref_frame = frame;
    que.push(frame);
    // keyframe_updated.notify_one();
}

int GraphOptimizer::addVertex(int Id, const g2o::Isometry3D &pose, int exisitId)
{
    
    history.push_back(Id);

    if (exisitId != -1) {
        sequences[Id] = find(exisitId);
        return n_vertices-1;
    }
    //Set up node
    g2o::VertexSE3 *v_se3 = new g2o::VertexSE3();
    v_se3->setId(Id);
    v_se3->setMarginalized(false);

    v_se3->setEstimate(pose);

    if (sequences.size() == 0)
    { //First node
        v_se3->setFixed(true);
    }
    // add to optimizer
    optimizer.addVertex(v_se3);
    sequences[Id] = Id;
    n_vertices++;
    return (n_vertices - 1);
}

void GraphOptimizer::addEdge(const int fromId, const int toId, const g2o::Isometry3D &rel_pose, const Eigen::Matrix<double,6,6> &information)
{

    g2o::VertexSE3* p1 = static_cast<g2o::VertexSE3*>(optimizer.vertex(sequences[fromId]));
    g2o::VertexSE3* p2 = static_cast<g2o::VertexSE3*>(optimizer.vertex(sequences[toId]));
    if (p1 == nullptr || p2 == nullptr)  {
        std::cout << fromId << ": " << sequences[fromId] <<", "<< toId << " "<< sequences[toId] << std::endl;
        std::cout << "Nul ptr detected" << std::endl;
        getchar();
    }
    
    int from = sequences[fromId];
    int to   = sequences[toId];

    std::pair<int, int> edge = {from, to};
    if (Edges.find(edge) != Edges.end()) {
        return;
    }
    Edges.insert(edge);

    g2o::EdgeSE3 *e_se3 = new g2o::EdgeSE3;
    e_se3->setVertex(0, optimizer.vertex(from));
    e_se3->setVertex(1, optimizer.vertex(to));
    e_se3->setMeasurement(rel_pose);
    e_se3->setRobustKernel( new g2o::RobustKernelHuber() );
    //Set the information matrix
    e_se3->setInformation(information);
    optimizer.addEdge(e_se3);
    // alledges.push_back(e_se3);
}

void  GraphOptimizer:: mainLoop() {

    while(!shutdownflag) {
        // cout <<"Hello from mainLoop"<< endl;
        
            // unique_lock<mutex> lck_update_frame(keyframe_update_mutex);
            // keyframe_updated.wait(lck_update_frame);
            que.pop();
            cout<<"keyframes are updated! Optimization starts!"<<endl;

            unique_lock<mutex> lck(keyframes_mutex);
            buffer = keyFrames;
            lck.unlock();

            for (int i = 0; i < buffer.size(); i++) {
                this->optimizer.vertex(sequences[history[i]])->setFixed(false);
            }   

            if (this->loopfound) {
                d_count = 0;
                cout << "Loop detected" << endl;
                //local optimization
                int large = buffer.size()/2;
                large = std::min(large, 2000);
                for (int i = 0; i < buffer.size() - large; i++) {
                    if (i < history.size() - large) {
                        optimizer.vertex(sequences[history[i]])->setFixed(true);
                    } else {
                        optimizer.vertex(sequences[history[i]])->setFixed(false);
                    }
                }
                this->optimizer.vertex(sequences[history[0]])->setFixed(true);

                 cout << "start initialization "<< endl;
                 optimizer.initializeOptimization();
                 optimizer.computeInitialGuess();
                 optimizer.computeActiveErrors();
                 optimizer.optimize(20);


            } else {
                    int large = 20;
                    for (int i = 0; i < buffer.size(); i++) {
                    if (i > large) {
                        optimizer.vertex(sequences[history[i]])->setFixed(true);
                    } 
                }
                this->optimizer.vertex(sequences[history[0]])->setFixed(true);
                optimizer.initializeOptimization();
                optimizer.computeInitialGuess();
                optimizer.computeActiveErrors();
                optimizer.optimize(10);
            }

            for (int i = 0; i < buffer.size(); i++) {
                g2o::VertexSE3* v_se3 = static_cast<g2o::VertexSE3*>(optimizer.vertex(sequences[buffer[i]->id]));
                g2o::Isometry3D T = v_se3->estimate();
                keyFrames[i]->setTransform(T);
            }
            usleep(500);
    }
}

int GraphOptimizer::find(int id) {
    if (sequences[id] == id) return id;
    return find(sequences[id]); 
}

