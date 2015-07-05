HOW TO OPERATE

To execute this program, please use the following steps:

1. Place the following files in a single directory:
  - utils.py (Helper function for converting input data to matrix form)
  - matrix.py (Various matrix processing functions)
  - finalproject.py (Contains our implementation of EKF leading to creation of final prediction output file)
  - An input text file of x,y measurements

2. From a command line, navigate to the directory in which you placed the files above.

3. Execute the following command (minus quotation marks): "python finalproject.py -t text_file"

4. The program will create an output file, "prediction.txt" of 60 frames of predicted motion in a list of x,y positions.


