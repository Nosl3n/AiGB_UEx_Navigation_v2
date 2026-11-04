#include <zstd.h>
#include <vector>
#include <stdexcept>
#include <cstdint>
#include <Lidar3D.h> // Assumes definitions such as TShortArray (vector<int16_t>) and TByteArray (vector<int8_t>)
#include <variant>
#include <omp.h>
#include <array>
#include <cstring> // For memcpy

// Aliases for common types, assuming the definitions in Lidar3D.h
using TShortArray = RoboCompLidar3D::TShortArray; // Usually vector<int16_t>
using TByteArray = RoboCompLidar3D::TByteArray;   // Usually vector<int8_t>
using TColorCloudData = RoboCompLidar3D::TColorCloudData;

class CloudCompressor {
private:
    // Avoid copies and assignments inside the compression loops
    // Use pointers to the structures' data to avoid the 'switch' inside the loop
    // and to be able to pass them to functions that handle raw data.

    template<typename InputArray, typename DeltaType>
    static void encodeDelta(const InputArray& input, std::vector<DeltaType>& output) {
        output.resize(input.size());
        if (input.empty()) return;

        // Optimisation: static_cast only once, then work with wider types (32-bit or 16-bit)
        DeltaType prevVal = static_cast<DeltaType>(input[0]);
        output[0] = prevVal;

        for (size_t i = 1; i < input.size(); ++i) {
            DeltaType currentVal = static_cast<DeltaType>(input[i]);
            output[i] = currentVal - prevVal;
            prevVal = currentVal;
        }
    }

    template<typename DeltaType, typename OutputArray>
    static void decodeDelta(const std::vector<DeltaType>& deltas, OutputArray& output) {
        // Assumes output.resize(deltas.size()) is called BEFORE decodeDelta
        if (deltas.empty()) return;

        DeltaType currentVal = deltas[0];
        output[0] = static_cast<typename OutputArray::value_type>(currentVal);

        for (size_t i = 1; i < deltas.size(); ++i) {
            currentVal += deltas[i];
            // Conversion and assignment
            output[i] = static_cast<typename OutputArray::value_type>(currentVal);
        }
    }

    // A single Zstd compression method (cleaner and DRY)
    static void zstdCompress(const void* src, size_t srcSize, TByteArray& dest, int level) {
        // Avoid resizing dest to ZSTD_compressBound() when it is not needed.
        // Zstd can write straight into the destination buffer.
        size_t bound = ZSTD_compressBound(srcSize);
        if (dest.size() < bound) {
             dest.resize(bound); // Resized only if the current size is not enough.
        }
        size_t cSize = ZSTD_compress(dest.data(), dest.size(), src, srcSize, level);
        if (ZSTD_isError(cSize)) throw std::runtime_error(ZSTD_getErrorName(cSize));
        dest.resize(cSize); // Shrink to the real size
    }

    // A single Zstd decompression method (cleaner and DRY)
    // Returns the real decompressed size
    static size_t zstdDecompress(const TByteArray& src, void* dest, size_t destSize) {
        size_t dSize = ZSTD_decompress(dest, destSize, src.data(), src.size());
        if (ZSTD_isError(dSize)) {
             throw std::runtime_error(std::string("ZSTD Decompress Error: ") + ZSTD_getErrorName(dSize));
        }
        return dSize;
    }


public:
    static void compress(TColorCloudData& in, int level = 1) {
        if (in.X.empty()) { 
            in.numberPoints = 0; 
        return; 
        }
        
        size_t n = in.numberPoints; // numberPoints must already be set

        // ----------------------------------------------------
        // MEMORY PRE-ALLOCATION (for the compressed buffers)
        // ----------------------------------------------------
        size_t bound32 = ZSTD_compressBound(n * sizeof(int32_t));
        in.cX.resize(bound32);
        in.cY.resize(bound32);
        in.cZ.resize(bound32);
        
        // The original colour fields (R, G, B) become the compressed buffers.
        size_t bound16 = ZSTD_compressBound(n * sizeof(int16_t));

        // As many threads as tasks (6), or fewer if the hardware imposes it
        #pragma omp parallel num_threads(6)
        {
            // Thread-local buffers for the delta encoding
            std::vector<int32_t> deltaXYZ; 
            std::vector<int16_t> deltaRGB;
            
            // Task assignment with #pragma omp sections (better for a small, fixed number of tasks)
            #pragma omp sections nowait
            {
                // Task 1: X
                #pragma omp section
                {
                    encodeDelta(in.X, deltaXYZ);
                    in.X.clear(); in.X.shrink_to_fit();
                    zstdCompress(deltaXYZ.data(), deltaXYZ.size() * sizeof(int32_t), in.cX, level);
                }
                
                // Task 2: Y
                #pragma omp section
                {
                    encodeDelta(in.Y, deltaXYZ);
                    in.Y.clear(); in.Y.shrink_to_fit();
                    zstdCompress(deltaXYZ.data(), deltaXYZ.size() * sizeof(int32_t), in.cY, level);
                }
                
                // Task 3: Z
                #pragma omp section
                {
                    encodeDelta(in.Z, deltaXYZ);
                    in.Z.clear(); in.Z.shrink_to_fit();
                    zstdCompress(deltaXYZ.data(), deltaXYZ.size() * sizeof(int32_t), in.cZ, level);
                }
                
                // Task 4: R
                #pragma omp section
                {
                    encodeDelta(in.R, deltaRGB);
                    in.R.resize(bound16);
                    zstdCompress(deltaRGB.data(), deltaRGB.size() * sizeof(int16_t), in.R, level);
                }

                // Task 5: G
                #pragma omp section
                {
                    encodeDelta(in.G, deltaRGB);
                    in.G.resize(bound16);
                    zstdCompress(deltaRGB.data(), deltaRGB.size() * sizeof(int16_t), in.G, level);
                }

                // Task 6: B
                #pragma omp section
                {
                    encodeDelta(in.B, deltaRGB);
                    in.B.resize(bound16);
                    zstdCompress(deltaRGB.data(), deltaRGB.size() * sizeof(int16_t), in.B, level);
                }
            } // end sections
        } // end parallel
        in.compressed = true;
    }

    static void decompress(TColorCloudData& in) {

       size_t n = in.numberPoints;
        if (n == 0) return;

        // ----------------------------------------------------
        // 1. Pre-allocation of the decompressed output buffers (the 8 MB)
        // ----------------------------------------------------
        in.X.resize(n); 
        in.Y.resize(n); 
        in.Z.resize(n);


        // ----------------------------------------------------
        // OpenMP for decompression (like compression)
        // ----------------------------------------------------
        #pragma omp parallel num_threads(6)
        {
            // Thread-local delta buffers, resized only once
            // **Optimisation:** std::array would give a fixed size and avoid the heap allocation of vector<T>(n)
            // inside the loop. But since 'n' is variable, it is better to keep 'std::vector' with 'reserve' and 'resize'
            // outside the parallel block, or do a thread-local 'resize'.
            std::vector<int32_t> deltaXYZ; 
            std::vector<int16_t> deltaRGB;
            
            deltaXYZ.resize(n);
            deltaRGB.resize(n);

            #pragma omp sections nowait
            {
                // Task 1: X
                #pragma omp section
                {
                    zstdDecompress(in.cX, deltaXYZ.data(), n * sizeof(int32_t));
                    in.cX.clear(); in.cX.shrink_to_fit();
                    decodeDelta(deltaXYZ, in.X);
                }
                
                // Task 2: Y
                #pragma omp section
                {
                    zstdDecompress(in.cY, deltaXYZ.data(), n * sizeof(int32_t));
                    in.cY.clear(); in.cY.shrink_to_fit();
                    decodeDelta(deltaXYZ, in.Y);
                }
                
                // Task 3: Z
                #pragma omp section
                {
                    zstdDecompress(in.cZ, deltaXYZ.data(), n * sizeof(int32_t));
                    in.cZ.clear(); in.cZ.shrink_to_fit();
                    decodeDelta(deltaXYZ, in.Z);
                }
                
                // Task 4: R
                #pragma omp section
                {
                    zstdDecompress(in.R, deltaRGB.data(), n * sizeof(int16_t));
                    in.R.resize(n);  
                    decodeDelta(deltaRGB, in.R);
                }

                // Task 5: G
                #pragma omp section
                {
                    zstdDecompress(in.G, deltaRGB.data(), n * sizeof(int16_t));
                    in.G.resize(n);  
                    decodeDelta(deltaRGB, in.G);
                }

                // Task 6: B
                #pragma omp section
                {
                    zstdDecompress(in.B, deltaRGB.data(), n * sizeof(int16_t));
                    in.B.resize(n);
                    decodeDelta(deltaRGB, in.B);
                }
            } // end sections
        } // end parallel


        in.compressed = false;
    }

};