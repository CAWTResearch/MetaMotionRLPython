import time
def main():
    start = time.perf_counter()
    signal = [0] * 1024
    for i in range(1024):
        signal[i] = i + 1

    kernel = [1, 1, 1]
    result = [0] * (len(signal) - len(kernel) + 1)
    for i in range(len(result)):
        sum = 0 
        for j in range(len(kernel)):
            sum+=signal[i + j] * kernel[j]
        result[i] = (sum)
    end = time.perf_counter() - start
    print(end)

    # i = len(result) - 4
    # print(f"{result[0:3]}")
    # print(f"{result[i:i+4]}")

if __name__ == "__main__":
    main()
   