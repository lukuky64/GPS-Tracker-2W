#include "ExpFilter.hpp"

ExpFilter::ExpFilter(float alpha, uint8_t numDimensions) : m_initialised(false), m_numDimensions(numDimensions) {
  setAlpha(alpha);
  m_filteredData = std::vector<float>(m_numDimensions, 0.0f);
}

void ExpFilter::processSample(const std::vector<float>& inputData) {
  if (inputData.size() != m_numDimensions) return;  // Ignore if size mismatch

  if (!m_initialised) {
    m_filteredData = inputData;
    m_initialised = true;
  } else {
    for (uint8_t i = 0; i < m_numDimensions; ++i) {
      m_filteredData[i] = m_alpha * inputData[i] + (1.0f - m_alpha) * m_filteredData[i];
    }
  }
}

std::vector<float> ExpFilter::getFilteredData() const { return m_filteredData; }
void ExpFilter::setAlpha(float alpha) {
  if (alpha < 0.0f || alpha > 1.0f) {
    alpha = 0.5f;  // Default value if out of range
  }
  m_alpha = alpha;
}

void ExpFilter::reset(const std::vector<float>& values) {
  if (values.size() == m_numDimensions) {
    m_filteredData = values;
    m_initialised = false;
  }
}

bool ExpFilter::isInitialised() const { return m_initialised; }