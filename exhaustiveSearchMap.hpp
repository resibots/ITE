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

#ifndef EXHAUSTIVE_SEARCH_ARCHIVE_HPP_
#define EXHAUSTIVE_SEARCH_ARCHIVE_HPP_
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
namespace limbo {

    namespace inner_optimization {
        template <typename Params>
        struct ExhaustiveSearchArchive {

            ExhaustiveSearchArchive() {}
            template <typename AcquisitionFunction>
            Eigen::VectorXd operator()(const AcquisitionFunction& acqui, size_t dim) const
            {
                Eigen::VectorXd result;
                float best_acqui = -INFINITY;

                typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;

                for (archive_it_t it = Params::archiveparams::archive.begin(); it != Params::archiveparams::archive.end(); ++it) {
                    Eigen::VectorXd temp(it->first.size());
                    for (size_t i = 0; i < it->first.size(); i++)
                        temp[i] = it->first[i];

                    float new_acqui = acqui(temp);
                    if (best_acqui < new_acqui || it == Params::archiveparams::archive.begin()) {
                        best_acqui = new_acqui;
                        result = temp;
                    }
                }
                std::cout << "NEW POINT, expected (GP): " << best_acqui << std::endl;
                return result;
            }

        private:
        };

        template <typename Params>
        class ExhaustiveSearchArchive_parallel {
        protected:
            typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;

            typedef std::vector<typename Params::archiveparams::archive_t::mapped_type> _map_vector_t;
            typedef typename _map_vector_t::iterator _map_vector_it_t;
            boost::shared_ptr<_map_vector_t> _map_vector_p;

        public:
            ExhaustiveSearchArchive_parallel()
            {
                _map_vector_p = boost::shared_ptr<_map_vector_t>(new _map_vector_t());
                for (archive_it_t it = Params::archiveparams::archive.begin(); it != Params::archiveparams::archive.end(); it++)
                    _map_vector_p->push_back(it->second);
            }

            template <typename AcquisitionFunction>
            Eigen::VectorXd operator()(const AcquisitionFunction& acqui, size_t dim)
            {
                Eigen::VectorXd result;
                Parallizer<AcquisitionFunction> para(acqui);

                tbb::parallel_reduce(tbb::blocked_range<_map_vector_it_t>(_map_vector_p->begin(), _map_vector_p->end()), para);

                Eigen::VectorXd temp(para.index_of_max->duty_cycle.size());
                for (int i = 0; i < para.index_of_max->duty_cycle.size(); i++)
                    temp[i] = para.index_of_max->duty_cycle[i];

                result = temp;
                return result;
            }

        protected:
            template <typename AcquisitionFunction>
            class Parallizer {
            public:
                Parallizer(const AcquisitionFunction& acqui) : value_of_max(-INFINITY), // FLT_MAX from <climits>
                                                               //	  index_of_max(_map_vector_p->end())
                                                               _acqui(acqui)
                {
                }

                Parallizer(Parallizer& x, tbb::split) : value_of_max(-INFINITY), // FLT_MAX from <climits>
                                                        _acqui(x._acqui) //	  index_of_max(-1)
                {
                    //	  _map_vector_p=x._map_vector_p;
                }

                void operator()(const tbb::blocked_range<_map_vector_it_t>& r)
                {
                    for (_map_vector_it_t it = r.begin(); it != r.end(); ++it) {

                        Eigen::VectorXd temp(it->duty_cycle.size());
                        for (int i = 0; i < it->duty_cycle.size(); i++)
                            temp[i] = it->duty_cycle[i];

                        float value = _acqui(temp);
                        if (value > value_of_max) {
                            value_of_max = value;
                            index_of_max = it;
                        }
                    }
                }
                void join(const Parallizer& y)
                {
                    if (y.value_of_max > value_of_max) {
                        value_of_max = y.value_of_max;
                        index_of_max = y.index_of_max;
                    }
                }

                float value_of_max;
                typename _map_vector_t::iterator index_of_max;

            protected:
                const AcquisitionFunction& _acqui;
            };
        };
    }
}
#endif
