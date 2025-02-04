from pyquaternion import Quaternion 


quat_1 = Quaternion(0.707, 0, 0, 0.707)
quat_2 = Quaternion(0.707, 0, 0, 0.707)

# Conjugar quat_1 y multiplicarlo por quat_2 para obtener el cuaternión relativo
relative_quaternion =  quat_2 * quat_1.conjugate

print(relative_quaternion)
print(type(quat_2))