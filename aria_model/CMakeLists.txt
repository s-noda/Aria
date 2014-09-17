project(aria_model)

cmake_minimum_required(VERSION 2.4.6)

execute_process(COMMAND sh ${PROJECT_SOURCE_DIR}/gen-sasabody.sh ${PROJECT_SOURCE_DIR})
