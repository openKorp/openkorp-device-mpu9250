#include "calibration.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

Calibration::Calibration()
  : m_center(0,0,0)
  , m_radius(0,0,0)
  , m_rotation(Eigen::Matrix3f::Zero())
{}

Calibration::Calibration(std::string const &a_file)
  : m_center(0,0,0)
  , m_radius(0,0,0)
  , m_rotation(Eigen::Matrix3f::Zero())
{
  if (loadCalibration(a_file) < 0) {
    std::cout << "Could not load " << a_file << ". Using default zero values." << std::endl;
  }
}


Calibration::Calibration(Eigen::Vector3f const &a_center, Eigen::Vector3f const &a_radius, Eigen::Matrix3f const &a_rotation)
  : m_center(a_center)
  , m_radius(a_radius)
  , m_rotation(a_rotation)
{}

Calibration::~Calibration()
{}

void Calibration::setCenter(Eigen::Vector3f const &a_center)
{
  m_center = a_center;
}
Eigen::Vector3f Calibration::getCenter() const
{
  return m_center;
}

void Calibration::setRadius(Eigen::Vector3f const &a_radius)
{
  m_radius = a_radius;
}

Eigen::Vector3f Calibration::getRadius() const
{
  return m_radius;
}

void Calibration::setRotation(Eigen::Matrix3f const &a_rotation)
{
  m_rotation = a_rotation;
}

Eigen::Matrix3f Calibration::getRotation() const
{
  return m_rotation;
}

int8_t Calibration::saveCalibration(std::string const &a_filename)
{
  Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ");
  std::ofstream file(a_filename);
  if (file.is_open()) {
    file << "# First row: center origin, second row: major axis scales, third row: rotation matrix" << std::endl;
    file << m_center.format(CSVFormat) << "\n" << m_radius.format(CSVFormat) << "\n" << m_rotation.format(CSVFormat) << "\n";
    std::cout << "[Calibration] Saved calibration to: " << a_filename << std::endl;
  } else {
    std::cout << "[Calibration] Unable to save calibration file. Tried to open: " + a_filename + "\n";
    return -1;
  }
  file.flush();
  file.close();
  return 0;
}

int8_t Calibration::loadCalibration(std::string const &a_filename) 
{
  std::ifstream file(a_filename, std::ifstream::in);
  if (file.is_open()){
    std::string line;
    // First line is a comment
    std::getline(file, line);

    // Center
    std::getline(file, line);
    std::vector<float> values;
    {
      std::stringstream lineStream(line);
      std::string cell;
      while (std::getline(lineStream, cell, ',')) {
        values.push_back(std::stof(cell));
      }
    }
    m_center = Eigen::Map<Eigen::Vector3f>(values.data());
    values.clear();

    // Axis radius
    std::getline(file, line);
    {
      std::stringstream lineStream(line);
      std::string cell;
      while (std::getline(lineStream, cell, ',')) {
        values.push_back(std::stof(cell));
      }
    }
    m_radius = Eigen::Map<Eigen::Vector3f>(values.data());
    values.clear();

    // Rotation
    std::getline(file, line);
    {
      std::stringstream lineStream(line);
      std::string cell;
      while (std::getline(lineStream, cell, ',')) {
        values.push_back(std::stof(cell));
      }
    }
    m_rotation = Eigen::Map<Eigen::Matrix3f>(values.data());
    values.clear();

    std::cout << "[Calibration] Loaded the existing calibration settings." << std::endl;
    file.close();
    return 0;
  } else {
    std::cout << "[Calibration] Could not load the calibration settings: " 
        << a_filename << std::endl;
    file.close();
    return -1;
  }
}

std::string Calibration::toString() 
{
  Eigen::IOFormat CleanFmt(Eigen::FullPrecision, 0, ", ", "\n", "[", "]");
  std::stringstream ss;
  ss << "[Calibration]\n";
  ss << "Center:\n";
  ss << m_center.format(CleanFmt) << "\n";
  ss << "Axis radius:\n";
  ss << m_radius.format(CleanFmt) << "\n";
  ss << "Rotation:\n";
  ss << m_rotation.format(CleanFmt) << "\n";

  return ss.str();
}

Eigen::Vector3f Calibration::applyOffset(Eigen::Vector3f const &a_data) const
{
  return a_data-m_center;

}
