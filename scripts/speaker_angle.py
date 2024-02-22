import math

s_x = .25
s_y = 5.55

print("Angle To Speaker Calculator.")
print("- enter robot position in \"x, y\" format.")
print("- enter \"exit\" to quit.")

while True:
    txt = input('>>> ')
    if txt == 'exit':
        break
    try:
        parts = txt.split(',')
        x = float(parts[0].strip())
        y = float(parts[1].strip())
        theta = math.atan2(s_y - y, s_x - x) + math.pi
        print(round(math.degrees(theta), 2))
    except: 
        print("Failed to parse, try again")
