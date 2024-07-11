#!/bin/sh

run_experiment()
{
  dataset_folder=$1
  ref_number=$2
  query_number=$3
  row_number=$4

  echo "\nEXPERIMENT USING REFERENCE ${ref_number} AND QUERY ${query_number} ON ROW ${row_number}:\n"
  echo "Producing the reference map..."
  make mapping dataset_main_folder:=${dataset_folder} ref_number:=${ref_number} row_number:=${row_number} >> /dev/null
  echo "DONE"
  echo "Producing the current map aligned with the reference..."
  make mapping_aligned dataset_main_folder:=${dataset_folder} ref_number:=${ref_number} query_number:=${query_number} row_number:=${row_number} >> /dev/null
  echo "DONE"
  echo "Computing visual associations with the reference map..."
  make associate dataset_main_folder:=${dataset_folder} ref_number:=${ref_number} query_number:=${query_number} row_number:=${row_number} >> /dev/null
  echo "DONE"
  echo "Running baseline deformation..."
  make cilantro_deform dataset_main_folder:=${dataset_folder} ref_number:=${ref_number} query_number:=${query_number} row_number:=${row_number} >> /dev/null
  echo "DONE"
  echo "Running my approach deformation..."
  make my_deform dataset_main_folder:=${dataset_folder} ref_number:=${ref_number} query_number:=${query_number} row_number:=${row_number} >> /dev/null
  echo "DONE"
  echo "Running evaluation..."
  echo "\nBASELINE RESULTS:"
  make evaluate_cilantro dataset_main_folder:=${dataset_folder} ref_number:=${ref_number} query_number:=${query_number} row_number:=${row_number} 
  echo "\nCOARSE REGISTRATION RESULTS:"
  make evaluate_superpoint dataset_main_folder:=${dataset_folder} ref_number:=${ref_number} query_number:=${query_number} row_number:=${row_number} 
  echo "\nMY APPROACH RESULTS:"
  make evaluate_myapproach dataset_main_folder:=${dataset_folder} ref_number:=${ref_number} query_number:=${query_number} row_number:=${row_number} 
  echo "\nDONE!"

  printf "%s " "\nPress ENTER to continue"
  read ans
  echo "\n"
}

dataset_folder_arg=$1
# Ref 1 Query 2
run_experiment ${dataset_folder_arg} 1 2 3
run_experiment ${dataset_folder_arg} 1 2 4
run_experiment ${dataset_folder_arg} 1 2 5

# Ref 1 Query 3
run_experiment ${dataset_folder_arg} 1 3 4

# Ref 1 Query 4
run_experiment ${dataset_folder_arg} 1 4 3
run_experiment ${dataset_folder_arg} 1 4 4
run_experiment ${dataset_folder_arg} 1 4 5

# Ref 1 Query 5
run_experiment ${dataset_folder_arg} 1 5 3
run_experiment ${dataset_folder_arg} 1 5 4
run_experiment ${dataset_folder_arg} 1 5 5

# Ref 1 Query 6
run_experiment ${dataset_folder_arg} 1 6 3
run_experiment ${dataset_folder_arg} 1 6 4
run_experiment ${dataset_folder_arg} 1 6 5

# Ref 1 Query 7
run_experiment ${dataset_folder_arg} 1 7 3
run_experiment ${dataset_folder_arg} 1 7 5

# Ref 2 Query 3
run_experiment ${dataset_folder_arg} 2 3 4

# Ref 2 Query 4
run_experiment ${dataset_folder_arg} 2 4 3
run_experiment ${dataset_folder_arg} 2 4 4
run_experiment ${dataset_folder_arg} 2 4 5

# Ref 2 Query 5
run_experiment ${dataset_folder_arg} 2 5 3
run_experiment ${dataset_folder_arg} 2 5 4
run_experiment ${dataset_folder_arg} 2 5 5

# Ref 2 Query 6
run_experiment ${dataset_folder_arg} 2 6 3
run_experiment ${dataset_folder_arg} 2 6 4
run_experiment ${dataset_folder_arg} 2 6 5

# Ref 2 Query 7
run_experiment ${dataset_folder_arg} 2 7 3
run_experiment ${dataset_folder_arg} 2 7 5

# Ref 3 Query 4
run_experiment ${dataset_folder_arg} 3 4 4

# Ref 3 Query 5
run_experiment ${dataset_folder_arg} 3 5 4

# Ref 3 Query 6
run_experiment ${dataset_folder_arg} 3 6 4

# Ref 4 Query 5
run_experiment ${dataset_folder_arg} 4 5 3
run_experiment ${dataset_folder_arg} 4 5 4
run_experiment ${dataset_folder_arg} 4 5 5

# Ref 4 Query 6
run_experiment ${dataset_folder_arg} 4 6 3
run_experiment ${dataset_folder_arg} 4 6 4
run_experiment ${dataset_folder_arg} 4 6 5

# Ref 4 Query 7
run_experiment ${dataset_folder_arg} 4 7 3
run_experiment ${dataset_folder_arg} 4 7 5

# Ref 5 Query 6
run_experiment ${dataset_folder_arg} 5 6 3
run_experiment ${dataset_folder_arg} 5 6 4
run_experiment ${dataset_folder_arg} 5 6 5

# Ref 5 Query 7
run_experiment ${dataset_folder_arg} 5 7 3
run_experiment ${dataset_folder_arg} 5 7 5

# Ref 6 Query 7
run_experiment ${dataset_folder_arg} 6 7 3
run_experiment ${dataset_folder_arg} 6 7 5


