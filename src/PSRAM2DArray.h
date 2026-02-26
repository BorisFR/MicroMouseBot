#ifndef PSRAM2DARRAY_H
#define PSRAM2DARRAY_H

#include "esp_heap_caps.h"

template <typename T>
class PSRAM2DArray
{
public:
    PSRAM2DArray(size_t r, size_t c) : data(nullptr), rows(r), cols(c)
    {
        if (rows > 0 && cols > 0)
        {
            data = static_cast<T *>(heap_caps_malloc(rows * cols * sizeof(T), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (!data)
            {
                Serial.println("[PSRAM2DArray] Allocation failed!");
                rows = cols = 0;
            }
            else
            {
                Serial.printf("[PSRAM2DArray] Allocated %u x %u elements in PSRAM\n", rows, cols);
            }
        }
    }

    // Destructor: free memory
    ~PSRAM2DArray()
    {
        if (data)
        {
            free(data);
            Serial.println("[PSRAM2DArray] Memory freed");
        }
    }

    // Access element with bounds checking
    T &at(size_t r, size_t c) const
    {
        if (r >= rows || c >= cols)
        {
            Serial.println("[PSRAM2DArray] Index out of range!");
            static T dummy{};
            return dummy;
        }
        return data[r * cols + c];
    }

    // Operator for row access (no bounds check)
    T *operator[](size_t r)
    {
        return &data[r * cols];
    }

    // Get dimensions
    size_t numRows() const { return rows; }
    size_t numCols() const { return cols; }

    // Fill all elements with a value
    void fill(const T &value)
    {
        if (data)
        {
            for (size_t i = 0; i < rows * cols; i++)
            {
                data[i] = value;
            }
        }
    }

    // Check if allocation succeeded
    bool isValid() const { return data != nullptr; }

    // Get total memory usage in bytes
    unsigned long allocationSize() const { return rows * cols * sizeof(T); }

private:
    T *data;     // Pointer to PSRAM memory
    size_t rows; // Number of rows
    size_t cols; // Number of columns
};

#endif // PSRAM2DARRAY_H