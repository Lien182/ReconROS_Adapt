import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

import numpy as np
import pandas as pd

df_rrinverse = pd.read_csv('/home/christian/git/ReconROS_Adapt/experiments/standard_executor_client/rr_inverse.csv', delimiter=';')
df_rrsobel = pd.read_csv('/home/christian/git/ReconROS_Adapt/experiments/standard_executor_client/rr_sobel.csv', delimiter=';')
df_rrsort = pd.read_csv('/home/christian/git/ReconROS_Adapt/experiments/standard_executor_client/rr_sort.csv', delimiter=';')
df_rrmnist = pd.read_csv('/home/christian/git/ReconROS_Adapt/experiments/standard_executor_client/rr_mnist.csv', delimiter=';')

df_inverse = pd.read_csv('/home/christian/git/ReconROS_Adapt/experiments/standard_executor_client/inverse.csv', delimiter=';')
df_sobel = pd.read_csv('/home/christian/git/ReconROS_Adapt/experiments/standard_executor_client/sobel.csv', delimiter=';')
df_sort = pd.read_csv('/home/christian/git/ReconROS_Adapt/experiments/standard_executor_client/sort.csv', delimiter=';')
df_mnist = pd.read_csv('/home/christian/git/ReconROS_Adapt/experiments/standard_executor_client/mnist.csv', delimiter=';')

#print(df_inverse)


#RR DATA
rrinverse_time = df_rrinverse['Inverse'][1001]
print(rrinverse_time)
rrinverse_data = df_rrinverse['Inverse'][1:1000]

rrmnist_time = df_rrmnist['Mnist'][1001]
print(rrmnist_time)
rrmnist_data = df_rrmnist['Mnist'][1:1000]

rrsobel_time = df_rrsobel['Sobel'][1001]
print(rrsobel_time)
rrsobel_data = df_rrsobel['Sobel'][1:1000]

rrsort_time = df_rrsort['Sort'][1000]
print(rrsort_time)
rrsort_data = df_rrsort['Sort'][1:1000]

#NONRR DATA
inverse_time = df_inverse['Inverse'][1001]
print(inverse_time)
inverse_data = df_inverse['Inverse'][1:1000]

mnist_time = df_mnist['Mnist'][1001]
print(mnist_time)
mnist_data = df_mnist['Mnist'][1:1000]

sobel_time = df_sobel['Sobel'][1001]
print(sobel_time)
sobel_data = df_sobel['Sobel'][1:1000]

sort_time = df_sort['Sort'][1000]
print(sort_time)
sort_data = df_sort['Sort'][1:1000]


fig, axs = plt.subplots(2, 2)
axs[0, 0].plot(inverse_data, 'tab:blue', label='Standard')
axs[0, 0].plot(rrinverse_data, 'tab:green', label='ReconROS')
axs[0, 0].legend()
axs[0, 0].set_title('Inverse')
axs[0, 1].plot(mnist_data, 'tab:blue', label='Standard')
axs[0, 1].plot(rrmnist_data, 'tab:green', label='ReconROS')
axs[0, 1].legend()
axs[0, 1].set_title('Mnist')
axs[1, 0].plot(sobel_data, 'tab:blue', label='Standard')
axs[1, 0].plot(rrsobel_data, 'tab:green', label='ReconROS')
axs[1, 0].legend()
axs[1, 0].set_title('Sobel')
axs[1, 1].plot(sort_data, 'tab:blue', label='Standard')
axs[1, 1].plot(rrsort_data, 'tab:green', label='ReconROS')
axs[1, 1].legend()
axs[1, 1].set_title('Sort')
plt.show()