#ifndef STAT_TRANSFERTS_HPP_
#define STAT_TRANSFERTS_HPP_


namespace limbo
{
  namespace stat
  {

    template<typename Params>
    struct StatTransferts : public Stat<Params>
    {
      std::ofstream _ofs;
      StatTransferts(){

      }

      template<typename BO>
      void operator()(const BO& bo)
      {
	this->_create_log_file(bo, "transferts.dat");	
	if (!bo.dump_enabled())
	  return;

	(*this->_log_file)<< bo.iteration()<<" : ";
	std::vector<float> sample;
	for(int i=0;i<bo.samples()[0].size();i++)
	  {
	    (*this->_log_file)<<bo.samples()[bo.samples().size()-1][i]<<" ";
	    sample.push_back(bo.samples()[bo.samples().size()-1][i]);
	  }

	(*this->_log_file)<<" : "<<Params::archiveparams::archive[sample].fit<<" : ";


	
	(*this->_log_file)<<bo.observations()[bo.observations().size()-1]<<std::endl;


	for(size_t i=0;i<Params::archiveparams::archive[sample].controller.size();i++)
	  (*this->_log_file)<<Params::archiveparams::archive[sample].controller[i]<<" ";
	(*this->_log_file)<<std::endl;
	(*this->_log_file)<<std::endl;
	
      }
    };
  }
}
#endif
