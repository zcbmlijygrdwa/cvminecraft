import matlab.engine
eng = matlab.engine.start_matlab()
tf = eng.isprime(37)
print(tf)

output = eng.addn(1,34)
print(output)
