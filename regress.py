table_string = """
shotTable.add(1.22, new Shot(3900, .175));
shotTable.add(1.75, new Shot(4100, .14));
shotTable.add(2.28, new Shot(4300, .12));
shotTable.add(2.77, new Shot(4500, .1));
shotTable.add(3.39, new Shot(4700, .085));
shotTable.add(4.1, new Shot(4800, .08));
shotTable.add(4.7, new Shot(5350, .069));
shotTable.add(5.4, new Shot(6050, .055));
"""

entries = list()
for raw_entry in table_string.split('\n'):
    if raw_entry == "": continue
    parse_entry = raw_entry.replace("shotTable.add(", "").replace("new Shot(", "").replace("));", "").replace(" ", "")
    entry = tuple([float(n) for n in parse_entry.split(',')])
    entries.append(entry)


for entry in entries:
    print(str(entry[0]) + "\t" + str(entry[1]))

print()

for entry in entries:
    print(str(entry[0]) + "\t" + str(entry[2] * 10000))