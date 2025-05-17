import pandas as pd
import ast

def reformat_csv(input_path: str, output_path: str, sep: str = None) -> None:
    """
    Reads a CSV file where the first column 'x' contains tuples of (x, y) values
    and the second column 'y' contains time values t. Splits the tuple into separate
    x and y columns, renames the time column to 't', and writes the reformatted
    data to a new CSV file with columns ['x', 'y', 't'].

    Parameters:
    - input_path: Path to the input CSV file.
    - output_path: Path where the output CSV will be saved.
    - sep: Delimiter used in the input file. If empty string or None, pandas will infer the separator.
    """
    # Interpret empty string as None to let pandas infer separator
    if sep == "":
        sep = None

    # Read the CSV with pandas. If sep is None, pandas will infer the separator.
    df = pd.read_csv(input_path, sep=sep)

    # Parse the 'x' column as tuples safely using ast.literal_eval
    df[['x_val', 'y_val']] = df['x'].apply(lambda s: pd.Series(ast.literal_eval(s)))

    # Rename the original time column 'y' to 't'
    df = df.rename(columns={'y': 't'})

    # Select and reorder columns
    df_out = df[['x_val', 'y_val', 't']]
    df_out.columns = ['x', 'y', 't']

    # Write to output CSV
    df_out.to_csv(output_path, index=False)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Reformat CSV with tuple x-values into separate x, y, and t columns."
    )
    parser.add_argument('-i', '--input', default='input.csv',
                        help='Input CSV file path (default: input.csv)')
    parser.add_argument('-o', '--output', default='output.csv',
                        help='Output CSV file path (default: output.csv)')
    parser.add_argument(
        '--sep', default=None,
        help='Delimiter of the input file (e.g., "," or "\t"). Use empty string or omit to auto-detect.'
    )
    args = parser.parse_args()

    print(f"Reading from {args.input}, writing to {args.output}, separator={args.sep or 'auto'}")

    reformat_csv('/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.2_V_ref=0.3/mpc_data_actual_path.csv',
                  '/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.2_V_ref=0.3/mpc_data_actual_path2.csv', '')
