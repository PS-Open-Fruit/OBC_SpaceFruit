import time
sourceContent : str
destContent : str
print("\033c")

while True:
    with open("downloads/0.jpg","rb") as source:
        sourceContent = source.read()
    with open("downloads/testimg-1.jpg" , "rb") as dst:
        destContent = dst.read()
    print(f"Simple check ,Size (SRC) : {len(sourceContent)}, (DST) : {len(destContent)} , (MATCH?) : {len(sourceContent) == len(destContent)}")
    fail = len(sourceContent) != len(destContent)
    for i in range(len(sourceContent)):
        if (i == len(destContent) - 1):
            print("Still no error")
            break
        if (sourceContent[i] != destContent[i]):
            print(f"\033[31m\033[1mError at {i}\033[0m")
            fail = True
            break
    if not fail:
        print("Check Pass")
        break
    else:
        print("Check Failed")
        break

# for i in sourceContent: