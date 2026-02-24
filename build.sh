#clear
# -mcpu bleeding_edge
FLAGS="-target wasm32-freestanding -DWASM -fno-entry -mcpu bleeding_edge --stack 65536 -O ReleaseSmall"
zig build-exe src/main.c $FLAGS
ls -lh *.wasm
