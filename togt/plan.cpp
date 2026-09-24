// Link against the existing TOGT drolib; export exact ascending coefficients.
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include "drolib/race/race_params.hpp"
#include "drolib/race/race_track.hpp"
#include "drolib/race/race_planner.hpp"

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: togt_plan PARAMS_DIR TRACK_YAML OUTPUT_CSV\n";
    return 2;
  }
  drolib::RaceParams params;
  if (!params.load(argv[1], "setups.yaml")) return 3;
  auto track = std::make_shared<drolib::RaceTrack>();
  if (!track->load(argv[2])) return 4;
  drolib::RacePlanner planner(params);
  if (!planner.planTOGT(track)) return 5;
  const auto trajectory = planner.getTrajectory();
  if (!trajectory.valid()) return 6;
  std::ofstream output(argv[3]);
  if (!output) return 7;
  output << "duration";
  for (const auto axis : {"x", "y", "z", "yaw"})
    for (int k = 0; k < 8; ++k) output << ',' << axis << '^' << k;
  output << '\n' << std::setprecision(17);
  for (int i = 0; i < trajectory.polys.getPieceNum(); ++i) {
    const auto& piece = trajectory.polys[i];
    output << piece.getDuration();
    const auto coefficients = piece.getCoeffMat();
    for (int axis = 0; axis < 3; ++axis)
      for (int k = 0; k < 8; ++k) output << ',' << coefficients(axis, 7-k);
    // Optimization uses ConstAngle(0). Keep exported heading consistent with it.
    for (int k = 0; k < 8; ++k) output << ",0";
    output << '\n';
  }
  std::cout << "TOGT solved: " << trajectory.polys.getPieceNum()
            << " pieces, " << trajectory.getTotalDuration() << " s\n";
  return output.good() ? 0 : 8;
}
