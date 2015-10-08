#ifndef RANDOM_SELECTION_HPP_
#define RANDOM_SELECTION_HPP_

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <bayesian_optimization/misc/rand.hpp>

namespace limbo
{

  namespace inner_optimization
  {
    template <typename Params>
    struct RandomSelection{

      RandomSelection(){}
      template <typename AcquisitionFunction>
      float operator()(const AcquisitionFunction& acqui,Eigen::VectorXd& result)const
      {

	int indiv=misc::rand(Params::archiveparams::archive.size()-1);
	typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;
	
	archive_it_t it=Params::archiveparams::archive.begin();
	std::advance(it,indiv);
	if(result.size()!=it->first.size())
	  result=Eigen::VectorXd(it->first.size());
	for(int i=0;i<it->first.size();i++)
	  result[i]=it->first[i];
	
	return it->second.fit;
      
      }

    private:

    };
  
  }
}




#endif
