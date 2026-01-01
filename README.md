# KHI: A Multi-Attribute RFANNS Index

This repository contains the C++ implementation of **KHI**, an index for multi-attribute range-filtering approximate nearest neighbor search (RFANNS) in Euclidean spaces. KHI combines an attribute-space partitioning tree with filtered HNSW graphs to support efficient $k$NN queries under numeric range predicates on multiple attributes.

For details of the algorithm and experiments, please refer to the associated paper and the technical report.

## 1. Requirements

- C++17-compatible compiler (e.g., `g++` or `clang++`)
- CMake ≥ 3.10
- Linux environment (tested on x86\_64)
- Recommended: multi-core CPU for parallel index construction and querying

(Optional) Datasets in the binary format described in `dataset/README.md`.


## 2. Build Instructions

```bash
mkdir build
cd build
cmake ..
make
```

This will produce the executable:

```
./build/KHI
```

## 3. Data Format

KHI expects five types of binary files per dataset:
- `<name>_meta.bin` – attribute metadata and statistics
- `<name>_vectors.bin` – dense vector embeddings for all objects
- `<name>_constraints_*_1k.bin` – range predicates for queries
- `<name>_query_vectors_1k.bin` – query embeddings
- `<name>_top100_*_1k.bin` – ground-truth $k$NN answers for evaluation

The exact formats of these files are documented in dataset/README.md.

## 4. Construction
```zsh
./build/KHI \
  -idx [attribute file] [embedding file] [index prefix] [num_threads] [M]
```

Arguments:
- attribute file

    Path to the attribute metadata file, e.g. `<name>_meta.bin`.
- embedding file

    Path to the base embedding file, e.g. `<name>_vectors.bin`.
- index prefix

    Directory in which the index files for KHI will be stored (created if needed).
- num_threads

    Number of worker threads used for index construction (e.g., 16).
- M

    Maximum degree bound for the filtered HNSW graphs (e.g., 32 as in the paper).

Example:
```zsh
./build/KHI \
  -idx data/laion_meta.bin \
       data/laion_vectors.bin \
       index/laion_32 \
       16 \
       32
```

## 5. Query
```zsh
./build/KHI \
  -q [attribute file] [embedding file] [index prefix] \
     [query embedding file] [query range file] \
     [groundtruth file] [log file] \
     [k] [M]
```

Arguments:
- attribute file

    Same `<name>_meta.bin` used at construction time.
- embedding file

    Same `<name>_vectors.bin` used at construction time.
- index prefix

    Directory containing the previously built index.
- query embedding file
  
    Query vectors, e.g. `<name>_query_vectors_1k.bin`.
- query range file

    Range predicates, e.g. `<name>_constraints_1_16_1k.bin`
- groundtruth file

    Top-100 ground-truth neighbors for evaluation, e.g. `<name>_top100_1_16_1k.bin`.
- log file

    Output path for logging statistics (QPS, recall, etc.).
- k

    Target number of nearest neighbors to return (e.g., 10).
- M

    Maximum degree bound during query (must match the index, e.g., 32).

Example:
```zsh
./build/KHI \
  -q data/laion_meta.bin \
     data/laion_vectors.bin \
     index/laion_32 \
     data/laion_query_vectors_1k.bin \
     data/laion_constraints_1_16_1k.bin \
     data/laion_top100_1_16_1k.bin \
     logs/laion_1_16.txt \
     10 \
     32
```

## 6. Reproducing Paper Experiments (Optional)
The main experiments in the paper can be reproduced by:

1.	Downloading the processed datasets (see dataset/README.md for links).

2.	Building KHI as described above.

3.	Running index construction with M = 32 and an appropriate number of threads.

4.	Running queries

5.	Viewing the recall and QPS reported in the log files.
