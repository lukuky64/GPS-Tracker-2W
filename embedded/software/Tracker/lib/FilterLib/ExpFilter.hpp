#pragma once
#include <Arduino.h>

class ExpFilter {
 public:
  // Constructor
  ExpFilter(float alpha = 0.5f, uint8_t numDimensions = 1);

  void setAlpha(float alpha);

  // Apply a filter to the input data
  void processSample(const std::vector<float>& inputData);
  std::vector<float> getFilteredData() const;

  void reset(const std::vector<float>& values);
  bool isInitialised() const;

 private:
  // keep between 0 (max smooth) and 1 (no smoothing)
  float m_alpha;
  std::vector<float> m_filteredData;
  bool m_initialised;
  uint8_t m_numDimensions;
  // Private members for filter parameters, if needed
};