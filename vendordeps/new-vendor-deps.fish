#!/usr/bin/env fish

set target_dir (dirname (status filename))
set year (date +%Y)

curl -o $target_dir/WPILibNewCommands.json 'https://raw.githubusercontent.com/wpilibsuite/allwpilib/main/wpilibNewCommands/WPILibNewCommands.json'

# Phoenix V5
curl -o $target_dir/Phoenix5.json 'https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc'{$year}'-latest.json'

# Phoenix V6
curl -o $target_dir/Phoenix6.json 'https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc'{$year}'-latest.json'

curl -o $target_dir/REVLib.json 'https://software-metadata.revrobotics.com/REVLib-'{$year}'.json'

curl -o $target_dir/navx_frc.json 'https://dev.studica.com/releases/'{$year}'/NavX.json'
