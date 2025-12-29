# KHI

## Usage
### Compile program
```zsh
mkdir build
cd build
cmake ..
make
```

### Construction
```zsh
./build/RFANN -idx [attribute file] [embedding file] [index storage path] [the number of threads] [M]
```

### Query
```zsh
./build/RFANN -q [attribute file] [embedding file] [index storage path] [query embedding file] [query range predicate file] [groundtruth file] [result file] [k] [M]
```
