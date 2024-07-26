# ONATS

*paper: Let Robots Watch Grass Grow: Optimal Task Assignment for Automatic Plant Factory*

## Abstract
Modularized plant factories, characterized by machines executing intelligent control requests to automatically take care of crops, have emerged as a sustainable agricultural paradigm, garnering the attention of Internet-of-Things and agricultural researchers for their production stability and energy efficiency. However, the diversity and pluralism of the plant factory components make it difficult to cooperate and produce crops with better qualities. Therefore, appropriate resource allocation and task scheduling strategies become the key points to optimize the quality of production in the factories by immediately telling which component is more suitable to do what in taking care of the crops. To address this challenge, this paper investigates how the machines of the factory can use their unique services and resource to help improve the crops' quality and model the machine cooperation as an online decision-making problem. An α-competitive approach called ONATS is designed based on the transformation of the original problem, and the experiments show that the proposed algorithm is superior to the baselines. Additionally, this paper explores the impact of different system configurations on the proposed method and shows that the proposed approach has broad applicability.

## Experimental parameters
![Experimental parameters](/figs/parameters.png)

## Experimental preparations
- install the MOSEK solver

## Experimental environment
![Experimental environment](/figs/robot.png)


## Experimental results
- profits of different approaches
![Real experiment result](/figs/value.png)

- running time of different approaches
![Real experiment result](/figs/time.png)

- the impact of the price-efficiency ratio of resources and the ratio of resource supply and demand
![Real experiment result](/figs/violin.png)

- resource utilization varies with the coming of control requests
![Real experiment result](/figs/heatmap.png)


