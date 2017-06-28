# coding=utf-8
'''
Created on 4/26/2017 2:08 28PM Wang Weimin

@author: wangwm
'''
import yaml

# 'file_name_digits': 4 #The number of digits of the filename


def default_params():
    '''Return default configuration
    '''
    default_params_yaml = open("config.yaml", "r")
    return yaml.load(default_params_yaml)

# print default_params()