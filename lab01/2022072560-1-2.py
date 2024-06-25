import numpy as np #numpy 라이브러리 import

M = np.arange(2, 27, 1) # arange 함수로 1만큼 간격으로 2~26까지 배열 생성

print(M) 
print("\n")

# A is done

M = M.reshape((5,5)) # reshape 함수로 1차원 배열 -> 5X5 행렬 

print(M) 
print("\n")

# B is done

M[1:4, 1:4] = 0 # 5X5 행렬의 inner 부분 0으로 변경 
print(M)
print("\n")

# C is done

M = M@M # 행렬 M에 대한 행렬곱
print(M)
print("\n")

# D is done

v = M[0, :] # 행렬 M의 1행을 벡터 v에 할당
v = np.sqrt(v@v) # '벡터의 내적' 후 크기를 구하기 위해 np.sqrt 함수 사용 
print(v)
