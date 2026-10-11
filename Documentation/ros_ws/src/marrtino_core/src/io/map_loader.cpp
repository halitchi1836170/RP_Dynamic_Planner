#include "marrtino_core/io/map_loader.hpp"

#include <cmath>
#include <fstream>
#include <locale>
#include <sstream>
#include <vector>

namespace marrtino::io {
namespace {

// Nel PGM scritto da map_server: libera = 254, occupata = 0, sconosciuta = 205.
constexpr unsigned char kUnknownPixel = 205;
constexpr double kFreePixel = 254.0;

bool fail(std::string* error, const std::string& message) {
  if (error != nullptr) *error = message;
  return false;
}

// Conversione numerica INDIPENDENTE DALLA LOCALE.
// std::stod segue la locale C corrente: con quella italiana attiva "0.02" si
// ferma al punto e restituisce 0. E' lo stesso inciampo per cui il C# usa
// CultureInfo.InvariantCulture ovunque. Qui si impone esplicitamente la locale
// "classica", cosi' il separatore decimale e' sempre il punto.
bool parseDouble(const std::string& text, double& out) {
  std::istringstream stream(text);
  stream.imbue(std::locale::classic());
  stream >> out;
  return !stream.fail();
}

std::string trim(const std::string& s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

struct MapMetadata {
  std::string image;
  double resolution{};
  double origin_x{};
  double origin_y{};
};

// Il YAML di map_server ha una forma fissa e nota: non serve una libreria,
// bastano le tre chiavi che ci interessano.
bool readYaml(const std::filesystem::path& path, MapMetadata& meta, std::string* error) {
  std::ifstream file(path);
  if (!file) return fail(error, "YAML non apribile: " + path.string());

  bool has_resolution = false;
  bool has_origin = false;

  std::string line;
  while (std::getline(file, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#') continue;

    if (line.rfind("image:", 0) == 0) {
      meta.image = trim(line.substr(6));
    } else if (line.rfind("resolution:", 0) == 0) {
      if (!parseDouble(trim(line.substr(11)), meta.resolution)) {
        return fail(error, "resolution illeggibile");
      }
      has_resolution = true;
    } else if (line.rfind("origin:", 0) == 0) {
      const auto lb = line.find('[');
      const auto rb = line.find(']');
      if (lb == std::string::npos || rb == std::string::npos || rb < lb) {
        return fail(error, "origin malformata");
      }
      std::istringstream inside(line.substr(lb + 1, rb - lb - 1));
      std::string token;
      // Solo i primi due valori: il terzo e' lo yaw, che qui non serve.
      if (!std::getline(inside, token, ',') || !parseDouble(trim(token), meta.origin_x)) {
        return fail(error, "origin.x illeggibile");
      }
      if (!std::getline(inside, token, ',') || !parseDouble(trim(token), meta.origin_y)) {
        return fail(error, "origin.y illeggibile");
      }
      has_origin = true;
    }
  }

  if (!has_resolution) return fail(error, "chiave resolution assente");
  if (!has_origin) return fail(error, "chiave origin assente");
  if (meta.resolution <= 0.0) return fail(error, "resolution non positiva");
  return true;
}

// Legge un token ASCII dell'header PGM, saltando spazi e commenti.
// I commenti ('#' fino a fine riga) sono ammessi dal formato: il C# non li
// gestiva, e un PGM prodotto da GIMP o da ImageMagick ne contiene uno.
bool readToken(const std::vector<unsigned char>& buf, std::size_t& pos, std::string& token) {
  const auto isSpace = [](unsigned char c) {
    return c == ' ' || c == '\n' || c == '\r' || c == '\t';
  };

  while (pos < buf.size()) {
    if (isSpace(buf[pos])) {
      ++pos;
    } else if (buf[pos] == '#') {
      while (pos < buf.size() && buf[pos] != '\n') ++pos;
    } else {
      break;
    }
  }
  if (pos >= buf.size()) return false;

  const std::size_t start = pos;
  while (pos < buf.size() && !isSpace(buf[pos]) && buf[pos] != '#') ++pos;
  token.assign(reinterpret_cast<const char*>(buf.data()) + start, pos - start);
  return true;
}

bool parseInt(const std::string& text, int& out) {
  std::istringstream stream(text);
  stream.imbue(std::locale::classic());
  stream >> out;
  return !stream.fail();
}

}  // namespace

std::optional<OccupancyGrid> loadOccupancyMap(const std::filesystem::path& yaml_path,
                                              std::string* error) {
  MapMetadata meta;
  if (!readYaml(yaml_path, meta, error)) return std::nullopt;

  // La chiave "image" e' relativa alla cartella del YAML. Se manca, si ripiega
  // sullo stesso nome base con estensione .pgm, che e' quello che fa il C#.
  std::filesystem::path pgm_path;
  if (meta.image.empty()) {
    pgm_path = yaml_path;
    pgm_path.replace_extension(".pgm");
  } else {
    pgm_path = yaml_path.parent_path() / meta.image;
  }

  // std::ios::binary e' obbligatorio: senza, su Windows ogni "\r\n" nei dati
  // dei pixel verrebbe tradotto in "\n" e il file arriverebbe SILENZIOSAMENTE
  // corrotto e piu' corto del previsto.
  std::ifstream file(pgm_path, std::ios::binary);
  if (!file) {
    if (error != nullptr) *error = "PGM non apribile: " + pgm_path.string();
    return std::nullopt;
  }

  const std::vector<unsigned char> buffer((std::istreambuf_iterator<char>(file)),
                                          std::istreambuf_iterator<char>());

  std::size_t pos = 0;
  std::string token;

  if (!readToken(buffer, pos, token) || token != "P5") {
    if (error != nullptr) *error = "magic PGM non e' P5 (atteso binario)";
    return std::nullopt;
  }

  int width = 0;
  int height = 0;
  int maxval = 0;
  if (!readToken(buffer, pos, token) || !parseInt(token, width) ||
      !readToken(buffer, pos, token) || !parseInt(token, height) ||
      !readToken(buffer, pos, token) || !parseInt(token, maxval)) {
    if (error != nullptr) *error = "header PGM illeggibile";
    return std::nullopt;
  }
  if (width <= 0 || height <= 0) {
    if (error != nullptr) *error = "dimensioni PGM non valide";
    return std::nullopt;
  }

  // Dopo maxval c'e' esattamente UN byte di whitespace, poi i dati binari.
  ++pos;

  const std::size_t expected = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (buffer.size() - pos < expected) {
    if (error != nullptr) {
      *error = "dati PGM troncati: attesi " + std::to_string(expected) + " byte, trovati " +
               std::to_string(buffer.size() - pos);
    }
    return std::nullopt;
  }

  const grid::GridGeometry geometry(width, height, meta.resolution,
                                    grid::Point2{meta.origin_x, meta.origin_y});
  OccupancyGrid map(geometry, static_cast<std::int8_t>(-1));

  // Ribaltamento verticale: il PGM ha origine in ALTO a sinistra con y verso il
  // basso, la griglia in BASSO a sinistra. La riga y del file diventa la riga
  // height-1-y della griglia. Dimenticarlo produce una mappa specchiata che
  // sembra plausibile finche' non la si guarda in RViz.
  for (int y = 0; y < height; ++y) {
    const std::size_t src_row = pos + static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
    const int dst_y = height - 1 - y;
    for (int x = 0; x < width; ++x) {
      const unsigned char pixel = buffer[src_row + static_cast<std::size_t>(x)];

      std::int8_t occupancy;
      if (pixel == kUnknownPixel) {
        occupancy = -1;
      } else {
        // Inversa della scrittura: 254 -> 0 (libera), 0 -> 100 (occupata).
        const double value = 100.0 * (kFreePixel - static_cast<double>(pixel)) / kFreePixel;
        occupancy = static_cast<std::int8_t>(std::lround(value));
      }

      map(grid::Cell{x, dst_y}) = occupancy;
    }
  }

  return map;
}

}  // namespace marrtino::io
