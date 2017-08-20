#ifndef sonarlog_target_tracking_ROC_HPP
#define sonarlog_target_tracking_ROC_HPP

#include <vector>

namespace sonarlog_target_tracking
{

class ROC
{

public:
    ROC(const std::vector<double>& true_positive_rate, const std::vector<double>& false_positive_rate);
    ~ROC();

private:

    void Calculate(const std::vector<double>& true_positive_rate, const std::vector<double>& false_positive_rate);
};

} // namespace sonarlog_target_tracking

#endif
