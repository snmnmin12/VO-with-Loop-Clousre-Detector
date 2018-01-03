
#ifndef _KEY_FRAME_SELECTION
#define _KEY_FRAME_SELECTION

#include "PlaceRecognition/PlaceDetector.h"
#include <memory>

using namespace std;

template<class TDescriptor, class F>
class TemplatedKeyFrameSelection {
public:
	template<class T>
	TemplatedKeyFrameSelection(const T& voc, float beta);
	virtual ~TemplatedKeyFrameSelection();

	bool process(const vector<TDescriptor>& features);

	inline void reset();
private:
	
	TemplatedVocabulary<TDescriptor, F>* m_voc;
	float m_beta;
	bool m_initialized;
  
    shared_ptr<DBoW2::BowVector> m_last;
    shared_ptr<DBoW2::BowVector> m_prev;
    shared_ptr<DBoW2::BowVector> m_vec;
};

template<class TDescriptor, class F>
template<class T>
TemplatedKeyFrameSelection<TDescriptor, F>::TemplatedKeyFrameSelection(const T& voc, float beta): 
m_beta(beta),m_initialized(false), m_last(nullptr), m_prev(nullptr), m_vec(nullptr) {
	m_voc = new T(voc);
}

template<class TDescriptor, class F>
TemplatedKeyFrameSelection<TDescriptor, F>::~TemplatedKeyFrameSelection() {
	delete m_voc;
}

template<class TDescriptor, class F>
bool TemplatedKeyFrameSelection<TDescriptor, F>::process(const vector<TDescriptor>& features) {

		m_vec.reset(new BowVector());
		m_voc->transform(features, *m_vec);

		bool ret;
		if (m_initialized) {
			double factor = m_voc->score(*m_prev, *m_vec);
			ret = ((m_voc->score(*m_last, *m_vec)/factor) < m_beta);
			m_prev = m_vec;
			if (ret) m_last = m_vec;
		} else {
			m_prev = m_vec;
			m_last = m_vec;
			m_initialized = true;
			ret = true;
		}
		return ret;
}

template<class TDescriptor, class F>
inline void TemplatedKeyFrameSelection<TDescriptor, F>::reset() {
	m_initialized = false;
}

#endif