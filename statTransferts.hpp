//| This file is a part of the ERC ResiBots project.
//| Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|                      Antoine Cully, cully@isir.upmc.fr
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#ifndef STAT_TRANSFERTS_HPP_
#define STAT_TRANSFERTS_HPP_

namespace limbo {
    namespace stat {

        template <typename Params>
        struct StatTransferts : public Stat<Params> {
            std::ofstream _ofs;
            StatTransferts()
            {
            }

            template <typename BO>
            void operator()(const BO& bo)
            {
                this->_create_log_file(bo, "transferts.dat");
                if (!bo.dump_enabled())
                    return;

                (*this->_log_file) << bo.iteration() << " : ";
                std::vector<float> sample;
                for (int i = 0; i < bo.samples()[0].size(); i++) {
                    (*this->_log_file) << bo.samples()[bo.samples().size() - 1][i] << " ";
                    sample.push_back(bo.samples()[bo.samples().size() - 1][i]);
                }

                (*this->_log_file) << " : " << Params::archiveparams::archive[sample].fit << " : ";

                (*this->_log_file) << bo.observations()[bo.observations().size() - 1] << std::endl;

                for (size_t i = 0; i < Params::archiveparams::archive[sample].controller.size(); i++)
                    (*this->_log_file) << Params::archiveparams::archive[sample].controller[i] << " ";
                (*this->_log_file) << std::endl;
                (*this->_log_file) << std::endl;
            }
        };
    }
}
#endif
