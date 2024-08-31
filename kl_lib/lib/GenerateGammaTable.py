gamma_name = "2dot8"
gamma = 2.8
max: int = 255
offset = 1  # will be added to all values save 0 one, with restriction to max.

print("static const uint8_t gamma_{}[] = {{".format(gamma_name))
for i in range(0, max + 1):
    g = 0 if i == 0 else offset + round(max * pow((i / max), gamma))
    g = g if g < max else max
    print("{: 4d},".format(g), end="\n" if (i + 1) % 16 == 0 else "")
print("};")