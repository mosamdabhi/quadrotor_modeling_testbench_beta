clc; clear all;  

yaml_map = yamlread('/home/mosam/Documents/quadrotor_modeling_testbench/matlab/utilities/TrajGen_Raw.yaml')

MappedParam = yaml_map.keys;
MappedValues = yaml_map.values;