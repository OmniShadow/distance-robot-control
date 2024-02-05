#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include <iostream>
#include <fstream>

class CsvLogger
{
private:
    const char *FILENAME;
    std::ofstream file;
    void createDirectories(const std::string& path);

public:
    CsvLogger(const std::string filename);
    ~CsvLogger();
    void flush();
    void write(const std::string header);
    CsvLogger &operator<<(const double new_val);
    void end_row();
    void close();
};

#endif