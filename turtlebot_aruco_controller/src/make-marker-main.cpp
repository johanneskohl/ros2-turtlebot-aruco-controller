#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>

#include <string>
#include <filesystem>
namespace fs = std::filesystem;
#include <iostream>
#include <getopt.h>


static void printHelp()
{
  std::cout <<
    "make-marker [--help] [--dictionary DICT] [--size SIZE] [--output FILE] MARKERID\n"
    "Create aruco marker for printing.\n"
    "\n"
    "Options:\n"
    "  --dictionary  -d  DICT  The OpenCV Aruco dictionary to choose the marker from. (default: 16; equivalent to cv::aruco::DICT_ARUCO_ORIGINAL)\n"
    "  --size        -s  SIZE  The border length of the aruco marker image. (default: 200)\n"
    "  --output      -o  FILE  The file to output the marker to. (default: marker-<MARKERID>.png)\n"
    "                          Note: The file extension '.png' will automatically be added (!) if it is missing.\n"
    "\n"
    "Positionals:\n"
    "  MARKERID                The ID of the marker inside the specified dictionary.\n";
}

int main(int argc, char *argv[])
{
  const option options[] = {
    option{
      .name = "dictionary",
      .has_arg = required_argument,
      .flag = nullptr,
      .val = 'd'
    },
    option{
      .name = "size",
      .has_arg = required_argument,
      .flag = nullptr,
      .val = 's'
    },
    option{
      .name = "output",
      .has_arg = required_argument,
      .flag = nullptr,
      .val = 'o'
    },
    option{
      .name = "help",
      .has_arg = no_argument,
      .flag = nullptr,
      .val = 'h'
    }
  };
  int result;
  int dictionaryId = cv::aruco::DICT_ARUCO_ORIGINAL,
      markerSize = 200;
  fs::path outputFileName;
  while ((result = getopt_long(argc, argv, "d:s:o:h", options, nullptr)) != -1)
  {
    switch (result)
    {
    case '?':
      printHelp();
      return 1;
    case 'h':
      printHelp();
      return 0;
    case 'd':
      dictionaryId = std::stoi(optarg);
      continue;
    case 's':
      markerSize = std::stoi(optarg);
      continue;
    case 'o':
      outputFileName = optarg;
      continue;
    }
  }
  int markerId;
  if (optind == (argc - 1))
  {
    markerId = std::stoi(argv[optind]);
  }
  else
  {
    std::cerr << "Missing required positional argument MARKERID!\n";
    printHelp();
    return 2;
  }
  if (outputFileName.empty())
  {
    outputFileName = "marker-" + std::to_string(markerId) + ".png";
  }
  else if (outputFileName.extension() != ".png")
  {
    // force png format
    outputFileName += ".png";
  }

  cv::Mat marker;
  try {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionaryId);
    cv::aruco::drawMarker(dictionary, markerId, markerSize, marker);
  }
  catch (const std::exception &exception)
  {
    std::cerr << "Failed to draw marker: " << exception.what() << '\n';
    return 3;
  }

  if (!cv::imwrite(outputFileName, marker))
  {
    std::cerr << "Failed to saving generated image to file.\n";
    return 4;
  }

  return 0;
}
