import numpy as np
import cvxpy as cp
import pandas as pd
import time
from scipy.optimize import fsolve
np.set_printoptions(suppress=True)


class SystemConf:
    def __init__(self, table):
        self.M = table['M']
        self.N = table['N']
        self.U = table['U']
        self.L = table['L']
        self.theta = self.U / self.L
        self.C = table['C']
        self.A = table['A']
        self.B = table['B']
        self.used_C = np.zeros(self.M)
        self.CR = {}
        self.beta = {}

    def __str__(self):
        return f'A = {self.A}, B = {self.B}, C = {self.C}'


# 定义请求类Request
class Request:
    def __init__(self, demand, max_rate, g):
        self.D = demand
        self.Y = max_rate
        self.g = g


def g_agg(y, conf: SystemConf, run_index):
    s = sum(y)
    return conf.A[run_index] * (1 - cp.exp(-s * 100))


def g_sep(y, conf: SystemConf, run_index, *args):
    s = 0
    if args[0]:
        for m in range(conf.M):
            s += conf.A[run_index] * cp.log(y[m] * conf.B[m] + 1)
    else:  # 求出真实的y之后，求g的值
        for m in range(conf.M):
            s += conf.A[run_index] * np.log(y[m] * conf.B[m] + 1)
    return s


def function_CR(alpha, conf: SystemConf):
    return alpha - 1 - 1 / (alpha - 1) - np.log((conf.theta * alpha - 1) / (alpha - 1))


def init_CR_beta(conf: SystemConf):
    ftype = 'ONATS'
    conf.CR[ftype] = fsolve(function_CR, 2, args=(conf))[0]
    conf.beta[ftype] = np.array([conf.C[m] / (conf.CR[ftype] - 1) for m in range(conf.M)])


def set_acc_const(conf: SystemConf):
    conf.Q_SEP_1 = (conf.U - conf.L) / (np.exp(conf.CR['ONATS']) - np.exp(conf.CR['ONATS'] / (conf.CR['ONATS'] - 1)))
    conf.Q_SEP_2 = np.array([conf.CR['ONATS'] / conf.C[m] for m in range(conf.M)])
    conf.Q_SEP_3 = conf.L / conf.CR['ONATS']


def find_opt(req: Request, conf: SystemConf, ftype='ONATS', run_index=0):
    objective = 0
    y = cp.Variable(conf.M)  # 构造M维向量y_n
    g_val = req.g(y, conf, run_index, True)
    objective += g_val  # 目标函数的第一部分

    for m in range(conf.M):
        if conf.used_C[m] < conf.beta[ftype][m]:
            objective -= conf.L * (conf.beta[ftype][m] - conf.used_C[m]) + conf.Q_SEP_1 / conf.Q_SEP_2[m] * (
                    cp.exp(conf.Q_SEP_2[m] * (conf.used_C[m] + y[m])) - np.exp(
                conf.Q_SEP_2[m] * conf.beta[ftype][m])) + conf.Q_SEP_3 * (
                                 conf.used_C[m] + y[m] - conf.beta[ftype][m])
        else:
            objective -= conf.Q_SEP_1 / conf.Q_SEP_2[m] * (
                    cp.exp(conf.Q_SEP_2[m] * (conf.used_C[m] + y[m])) - np.exp(
                conf.Q_SEP_2[m] * conf.used_C[m])) + conf.Q_SEP_3 * y[m]

    max_obj = cp.Maximize(objective)
    constraints = [0 <= y, sum(y) <= req.D]
    for m in range(conf.M):
        constraints += [conf.used_C[m] + y[m] <= conf.C[m]]
    for m in range(conf.M):
        constraints += [y[m] <= req.Y[m]]
    prob = cp.Problem(max_obj, constraints)
    prob.solve(solver=cp.MOSEK)

    each_ret = req.g(y.value, conf, run_index, False)
    return y.value, each_ret


def ONATS(req: Request, conf: SystemConf, ftype='ONATS', run_index=0):
    yn, each_ret = find_opt(req, conf, ftype, run_index)
    for m in range(conf.M):
        conf.used_C[m] += yn[m]
    conf.used_C = np.round(conf.used_C, 2)
    return np.round(yn, 2), np.round(each_ret, 2)


def balance(req: Request, conf: SystemConf, run_index=0):
    free_space = conf.C - conf.used_C
    # 修改精度误差
    for i in range(conf.M):
        if free_space[i] < 0:
            free_space[i] = 0
            conf.used_C[i] = conf.C[i]

    for i in range(conf.M):
        if free_space[i] > req.Y[i]:
            free_space[i] = req.Y[i]
    free_space_sum = sum(free_space)
    if free_space_sum == 0:  # 防止除零
        return np.zeros(conf.M), 0
    if free_space_sum < req.D:  # 背包剩余空间小于请求资源大小
        req.D = free_space_sum
    yn = req.D * free_space / free_space_sum
    each_ret = g_sep(yn, conf, run_index, False)
    conf.used_C += yn
    return np.round(yn, 2), np.round(each_ret, 2)


class Adaptor():
    def __init__(self, robot_num, service_num):
        self.R = robot_num
        self.K = service_num
        self.M = self.R * self.K

    def encode(self, m):
        r = m // self.K
        k = m % self.K
        return r, k

    def decode(self, r, k):
        return r * self.K + k


class IFRequest(Request):
    def __init__(self, demand, max_rate, g, service_type):
        super().__init__(demand, max_rate, g)
        self.service_type = service_type


class Robot():
    def __init__(self, index):
        self.index = index

    def run(self, task, time):
        print(f'机器人{self.index}正在执行任务{task}，需要时间{time}')


class IntelligentPlantFactory():
    def __init__(self, factory):
        self.robot_num = factory['robot_num']  # R
        self.service_num = factory['service_num']  # K
        self.knapsack = self.robot_num * self.service_num  # M
        self.request_num = factory['request_num']  # N
        self.adaptor = Adaptor(self.robot_num, self.service_num)
        self.robots = []  # 存放所有的机器人对象

    def read_file(self, filename, columns):
        column_names = [i for i in range(0, columns)]  # 列数不小于实际数据的列数
        data = pd.read_csv(filename, names=column_names, header=None)  # 一定要加入names参数，否则每行的列数不同会报错
        self.A = np.array(data.iloc[0])[:self.request_num]
        self.B = np.array(data.iloc[1])[:self.knapsack]  # A B 是g的参数
        self.C = np.array(data.iloc[2])[:self.knapsack]
        self.D = np.array(data.iloc[3])[:self.request_num]
        self.service_types = np.array(data.iloc[4])[:self.request_num].astype(int)  # service_types必须是int
        self.Y = []  # 速率限制
        for i in range(self.request_num):
            self.Y.append(np.array(data.iloc[5 + i])[:self.robot_num])

    def produce_args(self, factory):
        self.A = np.random.randint(1, factory['A_baseline'], size=self.request_num)
        self.B = np.random.randint(1, factory['B_baseline'], size=factory['robot_num'] * factory['service_num'])
        self.C = np.random.randint(100, 300, size=factory['robot_num'] * factory['service_num'])
        self.D = np.random.randint(1, factory['D_baseline'], size=factory['request_num'])
        self.service_types = np.random.randint(0, factory['service_num'], size=factory['request_num'])
        self.Y = []
        for i in range(self.request_num):
            self.Y.append(np.random.randint(0, factory['C_baseline'], size=factory['robot_num']))

    def update_LU(self):
        # U、L是g的偏导数上下界，所以与参数A、B、D有关，L 是A*B/(B*D+1)的最小值，U是A*B的最大值
        self.U = max(self.A) * max(self.B)
        self.L = min(self.A) * min(self.B) / (min(self.B) * max(self.D) + 1)

    def generate_request(self, demand, service_type, run_index, g_name):
        M = self.adaptor.M
        max_rate = np.zeros(M)  # max_rate 是一个M维向量
        for i in range(self.robot_num):
            max_rate[self.adaptor.decode(i, service_type)] = self.Y[run_index][i]
        request = IFRequest(demand, max_rate, g_name, service_type)
        return request

    def exec_request(self, req: IFRequest, conf: SystemConf, ftype, run_index):
        return ONATS(req, conf, ftype, run_index)  # ONATS() 返回分配向量 yn

    def assign(self, yn, service_type):  # 此时yn是 M = R*K 维，需要 encode
        for i in range(self.robot_num):
            m = self.adaptor.decode(i, service_type)
            self.robots[i].run(service_type, yn[m])

    def run(self, conf: SystemConf, ftype):
        all_sum = 0
        for i in range(self.request_num):
            time.sleep(1)
            request = self.generate_request(self.D[i], self.service_types[i], i, g_sep)
            yn, each_ret = self.exec_request(request, conf, ftype, i)
            all_sum += each_ret
            self.assign(yn, self.service_types[i])
        return all_sum


def write_file(filename, data):
    column_names = [i for i in range(0, len(data))]
    df = pd.DataFrame(columns=column_names, index=[])
    df.loc[0] = data
    df.index = df.index + 1
    df.to_csv(filename, mode='a', index=False, header=0)


def update_run(factory):
    plantfactory = IntelligentPlantFactory(factory)
    plantfactory.produce_args(factory)
    plantfactory.update_LU()

    table = {
        'M': plantfactory.adaptor.M,
        'N': plantfactory.request_num,
        'U': plantfactory.U,
        'L': plantfactory.L,

        'A': plantfactory.A,
        'B': plantfactory.B,
        'C': plantfactory.C,
    }
    conf = SystemConf(table)
    init_CR_beta(conf)
    set_acc_const(conf)
    sum_ONATS = plantfactory.run(conf, 'ONATS')
    write_file('results.txt', sum_ONATS)


if __name__ == "__main__":
    factory = {
        'robot_num': 10,  # R
        'service_num': 10,  # K
        'request_num': 300,  # N
        'A_baseline': 100,
        'B_baseline': 10,
        'C_baseline': 300,
        'D_baseline': 150,
    }
    update_run(factory)

