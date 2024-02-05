aggiungere le seguente istruzioni al CMakeLists.txt del proprio progetto per usare la libreria


add_subdirectory(distance_sensor)
target_link_libraries(%nome_progetto% PRIVATE distance_sensor)