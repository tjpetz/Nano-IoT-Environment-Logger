/**
 * Simple moving average filter.  It takes template parameters of the data type
 * as well as the Number of Taps (or size of the buffer).
 */

template <class DataType, unsigned int NbrOfTaps>
class MovingAverageFilter {
  
  public:
    MovingAverageFilter():
      bufferIndex(0)
      {
        memset(buffer, 0, sizeof(buffer));
      }

    /** @brief Add a sample to the filer buffer */
    void addSample(DataType s) {
      buffer[bufferIndex] = s;
      bufferIndex = (bufferIndex + 1) % NbrOfTaps;
    }

    /** @brief Compute the current moving average across the buffer */
    const DataType value() const {
      DataType sum = 0;
      for (auto i = 0; i < NbrOfTaps; i++) {
        sum += buffer[i];
      }
      return sum / NbrOfTaps;
    }

    /** @return The length of the sample buffer */
    unsigned int nbrOfTaps() {
      return NbrOfTaps;
    }

  private:
    DataType buffer[NbrOfTaps];
    unsigned int bufferIndex;
};