# OnlineRoutingParallel

This repository contains data and code associated with the paper "Online Routing over Parallel Networks: Deterministic Limits and Data-driven Enhancements" submitted to the Informs Journal on Computing.

## Data Sources and Format
All data sources are either .xlsx or .csv files and were obtained from several sources:

1. TomTom: Travel time information for routes between San Jose and San Francisco for the case study. The data consists of information on both highway and inner city routes and provides inforamtion on the distance and speed along different segments of each of those routes. Furthermore, this data-set also consists of information on the distribution of travel times on each of those routes, which are used to calibrate the travel time parameters of the model considered in our study.
2. CalTrans' PeMS Database: Flow rate information of users along the three main highways between San Jose and San Francisco. The flow rate information is given as the total number of vehicles passing a particular loop detector over each 5 minute time span from 5am-12pm and this is used to calibrate the arrival rate information in our study.
3. Statistical Atlas: Household income levels in the Bay Area, which are used to calibrate the values of time of users. The incomes levels represent the average income of the population in each census tract by year.

The travel time, arrival rate, and household income inforamtion from these data sources is compiled into the file "Aggregated_Data_Experiments.xlsx" in the folder "CompiledData".

## Code
The code used for the project is available in the folder named "Code". In particular, the code includes the implementation of the the greedy and scenario approach based algorithms proposed in the work as well as two benchmark approaches to test the efficacy of the algorithms developed in this work.
