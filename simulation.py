import os
import subprocess
import json
import pandas as pd

# Experiment parameters


# Result Formatting
column_names = [
    "mldSuccPrLink1", "mldSuccPrLink2", "mldSuccPrTotal", 
    "mldThptLink1", "mldThptLink2", "mldThptTotal", 
    "mldMeanQueDelayLink1", "mldMeanQueDelayLink2", "mldMeanQueDelayTotal", 
    "mldMeanAccDelayLink1", "mldMeanAccDelayLink2", "mldMeanAccDelayTotal", 
    "mldMeanE2eDelayLink1", "mldMeanE2eDelayLink2", "mldMeanE2eDelayTotal", 
    "mldSecondRawMomentAccDelayLink1", "mldSecondRawMomentAccDelayLink2", "mldSecondRawMomentAccDelayTotal", 
    "mldSecondCentralMomentAccDelayLink1", "mldSecondCentralMomentAccDelayLink2", "mldSecondCentralMomentAccDelayTotal", 
    "rngRun", "simulationTime", "payloadSize", 
    "mcs", "mcs2", "channelWidth", "channelWidth2", 
    "nMldSta", "mldPerNodeLambda", "mldProbLink1", 
    "mldAcLink1Int", "mldAcLink2Int", 
    "acBECwminLink1", "acBECwStageLink1", 
    "acBKCwminLink1", "acBKCwStageLink1", 
    "acVICwminLink1", "acVICwStageLink1", 
    "acVOCwminLink1", "acVOCwStageLink1", 
    "acBECwminLink2", "acBECwStageLink2", 
    "acBKCwminLink2", "acBKCwStageLink2", 
    "acVICwminLink2", "acVICwStageLink2", 
    "acVOCwminLink2", "acVOCwStageLink2"
]

output_file = 'fp_output.csv'

"""Load checkpoint from resume.json, if it exists"""
def load_checkpoint():
    if os.path.isfile('resume.json'):
        try:
            with open('resume.json', 'r') as file:
                print("Found resume.json. Resuming.")
                return json.load(file)
        except json.JSONDecodeError:
            print("Error decoding resume.json. Starting from scratch.")
    return {"sta": 0, "lam": -1}

"""Save the current checkpoint to resume.json."""
def save_checkpoint(sta_index, lam_index):
    checkpoint = {"sta": sta_index, "lam": lam_index}
    with open('resume.json', 'w') as file:
        json.dump(checkpoint, file, indent=4)

"""Run the simulation for the given STA and lambda value."""
def run_simulation(enable_emlsr, mcs1 = 6, mcs2 = 6, lam = -1, channelWidth = 20, channelWidth2 = 20):
    lambda_val = 10 ** lam
    mldProbLink1 = 0.5
    cmd = (
        f"./ns3 run 'single-bss-mld "
        f"--rngRun={42} "
        f"--payloadSize={2048} "
        f"--enable_emlsr={enable_emlsr} "
        f"--nMldSta={10} "
        f"--mldPerNodeLambda={lambda_val} "
        f"--mcs={mcs1} "
        f"--mcs2={mcs2} "
        f"--channelWidth={channelWidth} "
        f"--mldProbLink1={mldProbLink1} "
        f"--channelWidth2={channelWidth2}'"
    )
    print(cmd)
    subprocess.run(cmd, shell=True, check=True)

"""Read and append results from wifi-mld.dat to the output csv"""
def process_results(emlsr):
    if os.path.isfile('wifi-mld.dat'):
        data = pd.read_csv('wifi-mld.dat', names=column_names)
        data['emlsr'] = emlsr
        if os.path.isfile(output_file):
            data.to_csv(output_file, mode='a', header=False, index=False)  # Append without writing headers
        else:
            data.to_csv(output_file, mode='w', header=True, index=False)  # Write with headers if file doesn't exist
        os.remove('wifi-mld.dat')
    else:
        print("wifi-mld.dat not found.")

def main():
    lambdas = [-2.5 + 0.5 * x for x in range(7)]
    for lambda_ in lambdas:
        emlsr = False
        run_simulation(emlsr, lam = lambda_)
        process_results(emlsr)
        emlsr = True
        run_simulation(emlsr, lam = lambda_)
        process_results(emlsr)


if __name__ == "__main__":
    main()
