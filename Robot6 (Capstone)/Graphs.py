import matplotlib.pyplot as plt


with open('Xerr.csv', newline='') as file:
    Xerr = []
    ind = []
    j = 0
    for line in file.readlines():
        ind.append(j)
        row = line.split(',')
        T = []
        for i in range(len(row)):
            T.append(float(row[i]))
        Xerr.append(T)
        j += 1

print(ind)
print(Xerr)

plt.plot(ind, Xerr)
plt.show()