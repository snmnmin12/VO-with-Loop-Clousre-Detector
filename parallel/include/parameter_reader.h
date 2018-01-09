#ifndef PARAMETER_READER_H
#define PARAMETER_READER_H
#include "common_headers.h"

struct CAMERA_INTRINSIC_PARAMETERS;

class ParameterReader
{
public:
    ParameterReader( string filename="./resources/parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            // try ../parameter.txt
            fin.open(filename);
            if (!fin)
            {
                cerr<<"parameter file does not exist."<<endl;
                return;
            }
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }
            int pos = str.find('#');
            if (pos != -1)
            {
                //从井号到末尾的都是注释
                str = str.substr(0, pos);
            }
            pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }

    template< class T >
    T getData( const string& key ) const
    {
        auto iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
        }
        return boost::lexical_cast<T>( iter->second );
    }

    CAMERA_INTRINSIC_PARAMETERS getCamera() const {

        static CAMERA_INTRINSIC_PARAMETERS camera;
        camera.fx = this->getData<double>("camera.fx");
        camera.fy = this->getData<double>("camera.fy");
        camera.cx = this->getData<double>("camera.cx");
        camera.cy = this->getData<double>("camera.cy");
        camera.d0 = this->getData<double>("camera.d0");
        camera.d1 = this->getData<double>("camera.d1");
        camera.d2 = this->getData<double>("camera.d2");
        camera.d3 = this->getData<double>("camera.d3");
        camera.d4 = this->getData<double>("camera.d4");
        camera.scale = this->getData<double>("camera.scale");

        return camera;
    }

public:
    map<string, string> data;
};

#endif // PARAMETER_READER_H
