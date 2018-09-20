#include <eigen/Dense>
#include <string>

class Calibration
{
  public:
    Calibration();
    Calibration(std::string const &);
    Calibration(Eigen::Vector3f const &, Eigen::Vector3f const &, Eigen::Matrix3f const &);
    ~Calibration();

    void setCenter(Eigen::Vector3f const &);
    Eigen::Vector3f getCenter() const;
    void setRadius(Eigen::Vector3f const &);
    Eigen::Vector3f getRadius() const;
    void setRotation(Eigen::Matrix3f const &);
    Eigen::Matrix3f getRotation() const;
    int8_t saveCalibration(std::string const &);
    int8_t loadCalibration(std::string const &);
    std::string toString();
    Eigen::Vector3f applyOffset(Eigen::Vector3f const &) const;
    void ellipsoidFit(Eigen::MatrixXf const &);
    

  private:
    Eigen::Vector3f m_center;
    Eigen::Vector3f m_radius;
    Eigen::Matrix3f m_rotation;
};