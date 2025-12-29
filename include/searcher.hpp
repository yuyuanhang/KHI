#pragma once 

#include <vector>
#include <cstdint>
#include <cstdlib>
#include "memory.hpp"

namespace searcher
{
    template <typename Block = uint64_t>
    struct Bitset
    {
    private:
        constexpr static int block_size = sizeof(Block) * 8;
        int nbytes;
        Block *data;

    public:
        explicit Bitset(int n)
            : nbytes((n + block_size - 1) / block_size * sizeof(Block)),
              data(static_cast<uint64_t*>(memory::align_mm<64>(nbytes)))
        {
            std::memset(data, 0, nbytes);
        }

        ~Bitset() { free(data); }

        void set(int i)
        {
            data[i / block_size] |= (Block(1) << (i & (block_size - 1)));
        }

        bool get(int i)
        {
            return (data[i / block_size] >> (i & (block_size - 1))) & 1;
        }

        void *block_address(int i) { return data + i / block_size; }
    };
}