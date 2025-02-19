#pragma once


#include <boost/heap/fibonacci_heap.hpp>
#include <graph_hash_of_mixed_weighted/graph_hash_of_mixed_weighted.h>
#include <graph_hash_of_mixed_weighted/two_graphs_operations/graph_hash_of_mixed_weighted_to_graph_v_of_v_idealID.h>
#include <graph_hash_of_mixed_weighted_read_for_GSTP.h>
// 实现DPBF算法后，记得取消该注释
// #include "graph_v_of_v_idealID_DPBF_only_ec.h" 



void test_graph_v_of_v_idealID_DPBF_only_ec_cpu() {
    cout << "start.. " << endl;
    /*iteration*/
    cout << "------------------------------------------------------------" << endl;
    for (int i = 0; i < 10; i++) {
        std::unordered_set<int> generated_group_vertices;
        graph_hash_of_mixed_weighted instance_graph, generated_group_graph;
        graph_hash_of_mixed_weighted_read_for_GSTP(
                R"(..\data\data)" + to_string(i) + ".csv", instance_graph,
                generated_group_graph, generated_group_vertices);
                
        cout << "iteration " << i << endl;

        unordered_map<int, int> vertexID_old_to_new;
        int v_num = graph_hash_of_mixed_weighted_num_vertices(instance_graph);
        int g_num = generated_group_vertices.size();
        for (int mm = 0; mm < v_num; mm++) {
            vertexID_old_to_new[mm] = mm;
        }
        graph_v_of_v_idealID v_instance_graph = graph_hash_of_mixed_weighted_to_graph_v_of_v_idealID(instance_graph,
                                                                                                     vertexID_old_to_new);
        for (int mm = v_num; mm < v_num + g_num; mm++) {
            vertexID_old_to_new[mm] = mm;
        }
        graph_v_of_v_idealID v_generated_group_graph = graph_hash_of_mixed_weighted_to_graph_v_of_v_idealID(
                generated_group_graph, vertexID_old_to_new);

            int cost = 0;
            // 实现DPBF算法后，记得取消该注释
            // int cost = graph_v_of_v_idealID_DPBF_only_ec(v_instance_graph,
            //                                              v_generated_group_graph,
            //                                              generated_group_vertices);
            cout << cost << endl;
            cout << "------------------------------------------------------------" << endl;

    }
}