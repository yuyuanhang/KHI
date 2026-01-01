# Binary dataset format

All binary files include:

- integers are stored as 32-bit `int`
- floating-point values are stored as 32-bit `float`
- strings are stored as: `int len` followed by `len` raw bytes (no `\0` terminator)

For a dataset with name `<name>` (e.g., `dblp`, `youtube`, `msmarco`, `laion`), we use five binary files:

- `<name>_meta.bin`
- `<name>_vectors.bin`
- `<name>_query_vectors_1k.bin`
- `<name>_constraints_*.bin`
- `<name>_top100_*.bin`

Below we describe the role and layout of each file type.

---

## 1. `<name>_vectors.bin` / `<name>_query_vectors_1k.bin`

Stores the dense vector embeddings of all objects.  
This file is read by `Data::load_vector` (`Query::load_queries`).

Binary layout:

1. `int n_v` — number of vectors (also number of objects `n`)
2. `int d` — embedding dimension
3. `float v_v[n_v * d]` — contiguous array of embedding coordinates

Vectors are stored row-major: the embedding of object `i` occupies
`v_v[i * d, ..., (i + 1) * d - 1]`.

---

## 2. `<name>_meta.bin`

Stores the per-object **numeric attributes** and their names.  
This file is read by `Data::load_attribute` in the code.

Logical contents:

- number of attributes `n_a`
- attribute names (`header[0, ..., n_a - 1]`)
- number of objects `n`
- attribute values for all objects

A typical layout is:

1. `int n_a` — number of attributes  
2. For each attribute `i = 0 .. n_a-1`  
   2.1 `int len` — length of the attribute name  
   2.2 `char[len]` — attribute name bytes  
3. `int n` — number of objects  
4. For each attribute `i = 0, ..., n_a - 1`  
   4.1 `float attr[n]` — values of attribute `i` for all objects

---

## 3. `<name>_constraints_*.bin`

Stores the **range predicates for queries**. 
The suffix 1_16_1k encodes the target selectivity (1/16) and the number of range predicates (1,000).
This file is read by `Query::load_constraints`.

Binary layout:

1. `int n_a` — number of scalar constraint columns  
   - In our datasets we use **low / high** bounds for each attribute,  
     so `n_a = 2 * data->n_a`.
2. For each column `i = 0, ..., n_a - 1`  
   2.1 `int len` — length of the column name  
   2.2 `char[len]` — column name bytes  

   Column names are of the form `<attr>_low` and `<attr>_high`, and are later matched against `Data::header` to reconstruct per-attribute ranges.
3. `int n_q` — number of range predicates
4. For each column `i = 0, ..., n_a - 1`  
   4.1 `float col[n_q]` — values of that scalar for all queries

In `Query::load_constraints`, these columns are first read into a temporary
array `a_v[j][i]` and then re-ordered so that for each query `i` we obtain
a vector

$$
q\textunderscore a[i] = [\text{attr}_0^\text{low}, \text{attr}_0^\text{high}, \text{attr}_1^\text{low}, \text{attr}_1^\text{high}
          \dots]
$$

which is the range predicate \(B\) used by the RFANNS query Q=(q, B).

---

## 4. `<name>_top100_*.bin`

Stores the **ground-truth nearest neighbors** for evaluation.

The file is a binary stream of 32-bit integers and 32-bit floats in the
following layout:

1. `int32 n_q`  
   Number of queries in this ground-truth file.

2. `int32 top_k`  
   Number of ground-truth neighbors per query (typically `100`).

3. For each query index `i = 0 .. n_q - 1`:
   - `int32 knn_id[top_k]`
     The object IDs of the top-`top_k` nearest neighbors of query `i`. IDs are **0-based** indices into the dataset, consistent with the order used in `<name>_vectors.bin` / `<name>_meta.bin`.
   - `float32 knn_dist[top_k]`  
     The corresponding distances from query `i` to each neighbor in `knn_id[i]`, using the same embedding-space distance as in the code (e.g., L2).
   
The suffix of the filename encodes the selectivity and the number of queries, and is shared with the corresponding constraint file. For example:
	•	dblp_top100_1_16_1k.bin
	•	dblp_constraints_1_16_1k.bin

belong to the same query set, where 1_16 indicates the target selectivity (σ = 1/16) and 1k indicates that there are 1000 queries.

Across different selectivities (e.g., 1_16_1k, 1_64_1k, 1_256_1k), the query vectors file is the same (e.g., dblp_query_vectors_1k.bin): only the range predicates (*_constraints_*.bin) and the corresponding ground-truth files (*_top100_*.bin) change.

---

These five files together are sufficient to reproduce our experiments:

- `<name>_meta.bin`  — attribute schema + values  
- `<name>_vectors.bin and <name>_query_vectors_1k.bin` — embedding vectors  
- `<name>_constraints_*.bin` — per-query numeric ranges  
- `<name>_top100_*.bin` — ground-truth top-100 neighbors

Due to file size limits on this repository, we only include the following files here:
`<name>_query_vectors_1k.bin`, `<name>_constraints_*.bin`, and `<name>_top100_*.bin`.

The full dataset files `<name>_meta.bin` and `<name>_vectors.bin` are available on Zenodo at:
https://zenodo.org/records/18093887 and https://zenodo.org/records/18106651.
