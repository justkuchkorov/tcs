import csv

filenames = ["final_A1.csv", "final_A2.csv", "final_B1.csv", "final_B2.csv", "final_C1.csv", "final_C2.csv"]

with open('thesis_results.csv', mode="w", newline='') as out_file:
    writer = csv.writer(out_file)

    writer.writerow(["file", "speed at 10 s", "peak slip", "0-100", "last time above 0.2"])

    for filename in filenames:

        speed_at_10s = 0.0
        peak_slip_ice = 0.0
        time_to_100 = "Did not reach 100 km/h"
        found_100 = False
        last_out = "Never Lost Control"
        last_t = 0.0
        try:
            with open(filename, mode='r') as f:
                reader = csv.reader(f)
                next(reader)

                for row in reader:
                    t = float(row[0])
                    v_car = float(row[1])
                    slip = float(row[4])

                    last_t = t

                    if 9.99 < t < 10.01:
                        speed_at_10s = round(v_car, 2)
                    if slip > peak_slip_ice:
                            peak_slip_ice = round(slip, 4)
                    if v_car >= 27.78 and found_100 == False:
                        time_to_100 = round(t, 2)
                        found_100 = True

                    if slip >= 0.2:
                        last_out = t

            if last_out == last_t:
                last_out = "still out at the end"

            writer.writerow([filename, speed_at_10s, peak_slip_ice, time_to_100, last_out])
            print(f"Processed {filename} successfully.")

        except FileNotFoundError:
            print(f"Skipping {filename}: File not found.")