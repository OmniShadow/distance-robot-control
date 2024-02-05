#include "CsvLogger.hpp"
#include <stdio.h>
#include <iostream>
#include <filesystem>
#include <cstdio>
// #define LOGGING_DISABLED TRUE

CsvLogger::CsvLogger(const std::string filename) : FILENAME(filename.c_str())
{
    createDirectories(filename);
    
    file.open(filename);
    if (!file.is_open())
    {
        printf("impossibile aprire il file di log. esco...");
        exit(1);
    }
    file.precision(5);
}

// Function to create directories in the given path
void CsvLogger::createDirectories(const std::string &path)
{
    namespace fs = std::filesystem;

    fs::path dirPath = fs::path(path).parent_path();
    if (!fs::exists(dirPath))
    {
        if (!fs::create_directories(dirPath))
        {
            std::cerr << "Error creating directories. Exiting..." << std::endl;
            exit(1);
        }
    }
}

CsvLogger::~CsvLogger()
{
    flush();
    file.close();
}

void CsvLogger::write(const std::string header)
{
    file << header;
   file.flush();
}

void CsvLogger::flush()
{
    file.flush();
}

CsvLogger &CsvLogger::operator<<(const double new_val)
{
#ifndef LOGGING_DISABLED
    // file << std::scientific << new_val << ',';
    file << new_val << ',';
#endif
    return *this;
}

void CsvLogger::end_row()
{
#ifndef LOGGING_DISABLED
    file << '\n';
#endif
}

void CsvLogger::close()
{
    flush();
    file.close();
}