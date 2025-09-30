#clear
FLAGS="-target wasm32-freestanding -DWASM -fno-entry --stack 65536 -O ReleaseSmall"
zig build-exe main.c $FLAGS
ls -lh *.wasm
