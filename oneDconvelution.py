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
    #0.0035114089999979115 on rasberi
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.0038747840000041833
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.002382355000008829
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.0025603719999480745
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.0043800739999824145
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.002398541999923509
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.00239507899993896
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.0023719509999864385
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.002491301999953066
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.0024093039999115717
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.002555208999979186
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.0023519349999787664
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.0024790079999092995
# Pi@raspberrypi:~/Desktop/MetaMotionRLPython $ python3 oneDconvelution.py 
# 0.003934629000013956
