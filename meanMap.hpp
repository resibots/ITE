#ifndef MEAN_ARCHIVE_HPP_
#define MEAN_ARCHIVE_HPP_
namespace limbo {
    namespace mean_functions {
        template <typename Params>
        struct MeanArchive_Map {
            MeanArchive_Map()
            {
            }
            template <typename GP>
            float operator()(const Eigen::VectorXd& v, const GP&) const
            {
                std::vector<float> key(v.size(), 0);
                for (int i = 0; i < v.size(); i++)
                    key[i] = v[i];
                return Params::archiveparams::archive.at(key).fit;
            }
        };
    }
}

#endif
