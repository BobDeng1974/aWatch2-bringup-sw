void main() {
unsigned char inp[8];
unsigned char oup;
while (read(0, inp, 8) == 8) {
oup = 0;
for (int i = 0; i < 8; i++) oup |= inp[i] & (1 << (7 - i));
printf("0x%02x, ", oup);
}
}
