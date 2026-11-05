#include <algorithm>
#include <bsoncxx/builder/basic/array.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mongocxx/client.hpp>
#include <mongocxx/exception/exception.hpp>
#include <mongocxx/options/find.hpp>
#include <mongocxx/pipeline.hpp>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

#include "sailbot_db.h"

using bsoncxx::builder::basic::kvp;
using bsoncxx::builder::basic::make_array;
using bsoncxx::builder::basic::make_document;
using Args     = std::vector<std::string>;
using Doc      = bsoncxx::document::value;
using DocView  = bsoncxx::document::view;
using Table    = std::vector<std::vector<std::string>>;
using Targets  = std::vector<std::pair<std::string, Doc>>;
using CsvCells = std::map<std::string, std::vector<std::string>>;

const std::string DB_NAME             = "sailbot_db";
const std::string UNTAGGED            = "untagged";
const std::string TIMESTAMP_PATTERN   = "^([0-9]{2}-[0-9]{2}-[0-9]{2})%20([0-9]{2})%3A([0-9]{2})%3A([0-9]{2})$";
const std::string VOYAGE_NAME_PATTERN = "^[A-Za-z0-9_-]{1,40}$";
const std::string RANGE_PATTERN =
  "^(?:20)?([0-9]{2}-[0-9]{2}-[0-9]{2})(?:[_T]([0-9]{2})(?::([0-9]{2})(?::([0-9]{2}))?)?)?$";
const std::string RANGE_FORMATS = "Accepted formats: 2026-08-21, 2026-08-21_22:46, 2026-08-21T22:46:04, 26-08-21_22";
const std::string NAME_RULE   = "Voyage names use letters, digits, _ and - (1 to 40 characters) and cannot be untagged";
const std::string TIME_FORMAT = "%Y-%m-%d %H:%M:%S";
const std::string FOLDER_FORMAT = "%Y%m%d_%H%M%S";
const std::string CSV_HEADER =
  "time,timestamp_raw,voyage,latitude,longitude,speed,heading,batt_voltage,"
  "batt_current,wind_speed,wind_direction,temp,ph,salinity";

constexpr std::int64_t DEFAULT_LIST_SIZE = 20;
constexpr std::size_t  PREVIEW_ROWS      = 20;
constexpr std::size_t  MAX_NUMBER_DIGITS = 9;
constexpr int          COORD_DECIMALS    = 4;
constexpr int          VALUE_DECIMALS    = 1;
constexpr int          CSV_PRECISION     = 10;
constexpr int          TM_YEAR_BASE      = 1900;
constexpr int          KEY_CANCEL        = 3;
constexpr int          KEY_BACKSPACE     = 8;
constexpr int          KEY_DELETE        = 127;

const std::vector<std::string> ROW_HEADER = {"#", "voyage", "time", "lat", "lon", "speed", "heading", "parts"};

struct CsvSource
{
    std::string              collection;
    std::string              array;
    std::vector<std::string> fields;
};

const std::vector<CsvSource> CSV_SOURCES = {
  {COLLECTION_BATTERIES, "batteries", {"voltage", "current"}},
  {COLLECTION_WIND_SENSORS, "windSensors", {"speed", "direction"}},
  {COLLECTION_TEMP_SENSORS, "tempSensors", {"temperature"}},
  {COLLECTION_PH_SENSORS, "phSensors", {"ph"}},
  {COLLECTION_SALINITY_SENSORS, "salinitySensors", {"salinity"}}};

class CliDB : public SailbotDB
{
public:
    explicit CliDB(const std::string & uri) : SailbotDB(DB_NAME, uri), client_(pool_->acquire()) {}
    mongocxx::database   database() { return (*client_)[db_name_]; }
    mongocxx::collection collection(const std::string & name) { return database()[name]; }
    using SailbotDB::activateVoyage;
    using SailbotDB::generateVoyageName;
    using SailbotDB::insertVoyage;
    using SailbotDB::renameActiveVoyage;

private:
    mongocxx::pool::entry client_;
};

struct Row
{
    bsoncxx::oid id;
    std::string  timestamp;
    std::string  voyage;
    std::string  lat;
    std::string  lon;
    std::string  speed;
    std::string  heading;
    int          parts = 0;
};

struct Session
{
    CliDB &          db;
    std::string      voyage;
    std::vector<Row> rows;
    bool             running = true;
};

struct Command
{
    std::string                                  name;
    std::string                                  top_usage;
    std::string                                  inside_usage;
    std::string                                  description;
    std::function<void(Session &, const Args &)> run;
};

struct VoyageStats
{
    std::string messages = "0";
    std::string first;
    std::string last;
};

const std::vector<Command> & commands();

#ifdef _WIN32
class SilentInput
{
public:
    static int readKey() { return _getch(); }
};
#else
class SilentInput
{
public:
    SilentInput() : is_terminal_(tcgetattr(STDIN_FILENO, &original_) == 0)
    {
        if (is_terminal_) {
            termios silent = original_;
            silent.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | ISIG);
            silent.c_cc[VMIN]  = 1;
            silent.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &silent);
        }
    }
    ~SilentInput()
    {
        if (is_terminal_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_);
        }
    }
    SilentInput(const SilentInput &)             = delete;
    SilentInput & operator=(const SilentInput &) = delete;
    static int    readKey() { return std::cin.get(); }

private:
    termios original_{};
    bool    is_terminal_;
};
#endif

std::string readPassword()
{
    const SilentInput input;
    std::string       password;
    for (int key = SilentInput::readKey(); key != '\n' && key != '\r' && key != EOF; key = SilentInput::readKey()) {
        if (key == KEY_CANCEL) {
            password.clear();
            break;
        }
        if (key != KEY_BACKSPACE && key != KEY_DELETE) {
            password.push_back(static_cast<char>(key));
            std::cout << '*' << std::flush;
        } else if (!password.empty()) {
            password.pop_back();
            std::cout << "\b \b" << std::flush;
        }
    }
    std::cout << std::endl;
    return password;
}

bool readLine(const std::string & prompt, std::string & line)
{
    std::cout << prompt << std::flush;
    return static_cast<bool>(std::getline(std::cin, line));
}

Args splitWords(const std::string & line)
{
    std::istringstream words(line);
    Args               args;
    for (std::string word; words >> word;) {
        args.push_back(word);
    }
    return args;
}

Args splitOn(const std::string & text, char separator)
{
    std::istringstream pieces(text);
    Args               parts;
    for (std::string piece; std::getline(pieces, piece, separator);) {
        if (!piece.empty()) {
            parts.push_back(piece);
        }
    }
    return parts;
}

bool confirm(const std::string & question)
{
    std::string answer;
    const bool  answered = readLine(question + " (y/n) ", answer);
    if (answered && (splitWords(answer) == Args{"y"} || splitWords(answer) == Args{"Y"})) {
        return true;
    }
    std::cout << "Cancelled" << std::endl;
    return false;
}

bool confirmWord(const std::string & word)
{
    std::string answer;
    if (readLine("Type " + word + " to confirm: ", answer) && splitWords(answer) == Args{word}) {
        return true;
    }
    std::cout << "Cancelled" << std::endl;
    return false;
}

void printUsage(const std::string & usage) { std::cout << "Usage: " << usage << std::endl; }

void printBorder(const std::vector<std::size_t> & widths)
{
    for (const std::size_t width : widths) {
        std::cout << '+' << std::string(width + 2, '-');
    }
    std::cout << '+' << std::endl;
}

void printCells(const std::vector<std::string> & cells, const std::vector<std::size_t> & widths)
{
    for (std::size_t i = 0; i < widths.size(); i++) {
        const std::string cell = i < cells.size() ? cells[i] : "";
        std::cout << "| " << cell << std::string(widths[i] - cell.size() + 1, ' ');
    }
    std::cout << '|' << std::endl;
}

void printTable(const std::vector<std::string> & header, const Table & rows)
{
    std::vector<std::size_t> widths;
    for (const std::string & title : header) {
        widths.push_back(title.size());
    }
    for (const std::vector<std::string> & row : rows) {
        for (std::size_t i = 0; i < row.size() && i < widths.size(); i++) {
            widths[i] = std::max(widths[i], row[i].size());
        }
    }
    printBorder(widths);
    printCells(header, widths);
    printBorder(widths);
    for (const std::vector<std::string> & row : rows) {
        printCells(row, widths);
    }
    printBorder(widths);
}

std::string formatUtc(std::chrono::system_clock::time_point time, const std::string & format)
{
    const std::chrono::sys_days       day = std::chrono::floor<std::chrono::days>(time);
    const std::chrono::year_month_day date{day};
    const std::chrono::hh_mm_ss       clock{std::chrono::floor<std::chrono::seconds>(time - day)};
    std::tm                           utc{};
    utc.tm_year = static_cast<int>(date.year()) - TM_YEAR_BASE;
    utc.tm_mon  = static_cast<int>(static_cast<unsigned>(date.month())) - 1;
    utc.tm_mday = static_cast<int>(static_cast<unsigned>(date.day()));
    utc.tm_hour = static_cast<int>(clock.hours().count());
    utc.tm_min  = static_cast<int>(clock.minutes().count());
    utc.tm_sec  = static_cast<int>(clock.seconds().count());
    std::ostringstream out;
    out << std::put_time(&utc, format.c_str());
    return out.str();
}

std::string decodeTime(const std::string & raw)
{
    static const std::regex pattern(TIMESTAMP_PATTERN);
    std::smatch             match;
    if (!std::regex_match(raw, match, pattern)) {
        return raw;
    }
    return "20" + match.str(1) + " " + match.str(2) + ":" + match.str(3) + ":" + match.str(4);
}

std::optional<std::string> encodeTime(const std::string & input)
{
    static const std::regex pattern(RANGE_PATTERN);
    std::smatch             match;
    if (!std::regex_match(input, match, pattern)) {
        return std::nullopt;
    }
    std::string encoded = match.str(1);
    encoded += match[2].matched ? "%20" + match.str(2) : "";
    encoded += match[3].matched ? "%3A" + match.str(3) : "";
    encoded += match[4].matched ? "%3A" + match.str(4) : "";
    return encoded;
}

bool isValidName(const std::string & name)
{
    static const std::regex pattern(VOYAGE_NAME_PATTERN);
    return name != UNTAGGED && std::regex_match(name, pattern);
}

std::string voyageLabel(const std::string & voyage) { return voyage.empty() ? UNTAGGED : voyage; }

std::string fixed(double value, int decimals)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(decimals) << value;
    return out.str();
}

std::optional<double> parseDecimal(const std::string & digits)
{
    try {
        return std::stod(digits);
    } catch (const std::exception &) {
        return std::nullopt;
    }
}

template <typename Element>
std::optional<double> readNumber(const Element & element)
{
    if (!element) {
        return std::nullopt;
    }
    switch (element.type()) {
        case bsoncxx::type::k_double:
            return element.get_double().value;
        case bsoncxx::type::k_int32:
            return static_cast<double>(element.get_int32().value);
        case bsoncxx::type::k_int64:
            return static_cast<double>(element.get_int64().value);
        case bsoncxx::type::k_decimal128:
            return parseDecimal(element.get_decimal128().value.to_string());
        default:
            return std::nullopt;
    }
}

template <typename Element>
std::string text(const Element & element)
{
    return bsonString(element).value_or("");
}

template <typename Element>
std::string keyOf(const Element & element)
{
    const auto key = element.key();
    return std::string(key.data(), key.size());
}

template <typename Element>
std::string numberCell(const Element & element, int decimals)
{
    const std::optional<double> value = readNumber(element);
    return value ? fixed(*value, decimals) : "";
}

template <typename Element>
std::string csvNumber(const Element & element)
{
    const std::optional<double> value = readNumber(element);
    std::ostringstream          out;
    if (value) {
        out << std::setprecision(CSV_PRECISION) << *value;
    }
    return out.str();
}

template <typename Element>
std::string dateText(const Element & element)
{
    if (!element || element.type() != bsoncxx::type::k_date) {
        return "";
    }
    return formatUtc(std::chrono::system_clock::time_point{element.get_date().value}, TIME_FORMAT);
}

template <typename Element>
std::string valueText(const Element & element, int decimals)
{
    switch (element.type()) {
        case bsoncxx::type::k_double:
        case bsoncxx::type::k_decimal128:
            return numberCell(element, decimals);
        case bsoncxx::type::k_int32:
            return std::to_string(element.get_int32().value);
        case bsoncxx::type::k_int64:
            return std::to_string(element.get_int64().value);
        case bsoncxx::type::k_bool:
            return element.get_bool().value ? "true" : "false";
        case bsoncxx::type::k_oid:
            return element.get_oid().value.to_string();
        case bsoncxx::type::k_date:
            return dateText(element);
        case bsoncxx::type::k_null:
            return "null";
        default:
            return bsonString(element).value_or("(" + bsoncxx::to_string(element.type()) + ")");
    }
}

int decimalsFor(const std::string & path)
{
    return path.ends_with("latitude") || path.ends_with("longitude") ? COORD_DECIMALS : VALUE_DECIMALS;
}

template <typename Element>
void flatten(const std::string & collection, const std::string & path, const Element & element, Table & table)
{
    if (element.type() == bsoncxx::type::k_document) {
        for (const bsoncxx::document::element & child : element.get_document().value) {
            flatten(collection, path + "." + keyOf(child), child, table);
        }
    } else if (element.type() == bsoncxx::type::k_array) {
        const bsoncxx::array::view items = element.get_array().value;
        if (items.empty()) {
            table.push_back({collection, path, "[]"});
        }
        for (const bsoncxx::array::element & child : items) {
            flatten(collection, path + "[" + keyOf(child) + "]", child, table);
        }
    } else {
        table.push_back({collection, path, valueText(element, decimalsFor(path))});
    }
}

void appendScope(bsoncxx::builder::basic::document & filter, const std::string & voyage)
{
    if (voyage == UNTAGGED) {
        filter.append(kvp("voyage", make_document(kvp("$exists", false))));
    } else if (!voyage.empty()) {
        filter.append(kvp("voyage", voyage));
    }
}

Doc scopeFilter(const std::string & voyage)
{
    bsoncxx::builder::basic::document filter;
    appendScope(filter, voyage);
    return filter.extract();
}

Doc stringTimestampFilter(const std::string & voyage)
{
    bsoncxx::builder::basic::document filter;
    filter.append(kvp("timestamp", make_document(kvp("$type", "string"))));
    appendScope(filter, voyage);
    return filter.extract();
}

// The same timestamp can hold separate uploads in different voyages (an upload replayed after switching voyages),
// so a listed message is matched by its timestamp and its voyage, never by its timestamp alone
Doc rowFilter(const std::vector<Row> & rows)
{
    std::map<std::string, std::set<std::string>> timestamps;
    for (const Row & row : rows) {
        timestamps[voyageLabel(row.voyage)].insert(row.timestamp);
    }
    bsoncxx::builder::basic::array branches;
    for (const auto & [voyage, values] : timestamps) {
        bsoncxx::builder::basic::array in;
        for (const std::string & timestamp : values) {
            in.append(timestamp);
        }
        bsoncxx::builder::basic::document branch;
        branch.append(kvp("timestamp", make_document(kvp("$in", in.view()))));
        appendScope(branch, voyage);
        branches.append(branch.extract());
    }
    return make_document(kvp("$or", branches.view()));
}

Targets sensorTargets(DocView filter)
{
    Targets targets;
    for (const std::string & name : SENSOR_COLLECTIONS) {
        targets.emplace_back(name, Doc(filter));
    }
    return targets;
}

bool voyageExists(Session & session, const std::string & name)
{
    mongocxx::collection voyages = session.db.collection(COLLECTION_VOYAGES);
    return voyages.count_documents(make_document(kvp("name", name))) > 0;
}

bool isActive(Session & session, const std::string & name) { return session.db.getActiveVoyage() == name; }

void printNoVoyage(const std::string & name)
{
    std::cout << "No such voyage: " << name << " (it is case-sensitive)" << std::endl;
}

void printActiveVoyage(Session & session)
{
    const std::string active = session.db.getActiveVoyage();
    std::cout << "Active voyage: " << (active.empty() ? "none (new data will be untagged)" : active) << std::endl;
}

void leaveVoyage(Session & session, const std::string & message)
{
    std::cout << message << std::endl;
    session.voyage.clear();
    session.rows.clear();
}

bool voyageGone(Session & session)
{
    if (session.voyage.empty() || session.voyage == UNTAGGED || voyageExists(session, session.voyage)) {
        return false;
    }
    leaveVoyage(session, "Voyage " + session.voyage + " was deleted or renamed, back at the top level");
    return true;
}

std::optional<std::string> targetVoyage(const Session & session, const Args & args, const std::string & usage)
{
    if (session.voyage.empty() && args.size() == 2) {
        return args[1];
    }
    if (!session.voyage.empty() && args.size() == 1) {
        return session.voyage;
    }
    if (session.voyage.empty()) {
        printUsage(usage);
    } else {
        std::cout << "You are in voyage " << session.voyage << ", type quit to leave it first" << std::endl;
    }
    return std::nullopt;
}

Row toRow(const DocView & doc)
{
    return Row{
      doc["_id"].get_oid().value,
      text(doc["timestamp"]),
      text(doc["voyage"]),
      numberCell(doc["latitude"], COORD_DECIMALS),
      numberCell(doc["longitude"], COORD_DECIMALS),
      numberCell(doc["speed"], VALUE_DECIMALS),
      numberCell(doc["heading"], VALUE_DECIMALS)};
}

std::vector<Row> findRows(Session & session, DocView filter, int order, std::int64_t limit)
{
    mongocxx::options::find options;
    options.sort(make_document(kvp("timestamp", order), kvp("_id", order)));
    if (limit > 0) {
        options.limit(limit);
    }
    std::vector<Row>     rows;
    mongocxx::collection gps = session.db.collection(COLLECTION_GPS);
    for (const DocView & doc : gps.find(filter, options)) {
        if (doc["_id"] && doc["_id"].type() == bsoncxx::type::k_oid) {
            rows.push_back(toRow(doc));
        }
    }
    return rows;
}

void fillParts(Session & session, std::vector<Row> & rows)
{
    const Doc               filter = rowFilter(rows);
    mongocxx::options::find options;
    options.projection(make_document(kvp("timestamp", 1), kvp("voyage", 1), kvp("_id", 0)));
    for (const std::string & name : SENSOR_COLLECTIONS) {
        std::set<std::pair<std::string, std::string>> present;
        mongocxx::collection                          collection = session.db.collection(name);
        for (const DocView & doc : collection.find(filter.view(), options)) {
            present.emplace(text(doc["timestamp"]), text(doc["voyage"]));
        }
        for (Row & row : rows) {
            row.parts += static_cast<int>(present.count({row.timestamp, row.voyage}));
        }
    }
}

std::vector<std::string> rowCells(std::size_t index, const Row & row)
{
    const std::string parts = std::to_string(row.parts) + "/" + std::to_string(SENSOR_COLLECTIONS.size());
    return {
      std::to_string(index + 1),
      voyageLabel(row.voyage),
      decodeTime(row.timestamp),
      row.lat,
      row.lon,
      row.speed,
      row.heading,
      parts};
}

void showRows(Session & session, std::vector<Row> rows)
{
    if (!rows.empty()) {
        fillParts(session, rows);
    }
    session.rows = std::move(rows);
    if (session.rows.empty()) {
        std::cout << "No messages" << std::endl;
        return;
    }
    Table table;
    for (std::size_t i = 0; i < session.rows.size(); i++) {
        table.push_back(rowCells(i, session.rows[i]));
    }
    printTable(ROW_HEADER, table);
}

std::optional<std::size_t> parseNumber(const std::string & digits)
{
    const bool numeric =
      !digits.empty() && digits.size() <= MAX_NUMBER_DIGITS &&
      std::all_of(digits.begin(), digits.end(), [](unsigned char c) { return std::isdigit(c) != 0; });
    if (!numeric) {
        return std::nullopt;
    }
    return std::stoul(digits);
}

std::optional<std::size_t> parseRowNumber(const std::string & digits, std::size_t count)
{
    const std::size_t number = parseNumber(digits).value_or(0);
    if (number < 1 || number > count) {
        std::cout << "Bad row number: " << digits << " (listed rows are 1-" << count << ")" << std::endl;
        return std::nullopt;
    }
    return number;
}

bool addRows(const std::string & piece, std::size_t count, std::set<std::size_t> & picks)
{
    const std::size_t                dash = piece.find('-');
    const std::optional<std::size_t> low  = parseRowNumber(piece.substr(0, dash), count);
    if (!low) {
        return false;
    }
    const std::optional<std::size_t> high =
      dash == std::string::npos ? low : parseRowNumber(piece.substr(dash + 1), count);
    if (!high) {
        return false;
    }
    if (*low > *high) {
        std::cout << "Bad row range: " << piece << std::endl;
        return false;
    }
    for (std::size_t number = *low; number <= *high; number++) {
        picks.insert(number - 1);
    }
    return true;
}

std::optional<std::vector<std::size_t>> parseRows(const Session & session, const Args & args, std::size_t first)
{
    if (session.rows.empty()) {
        std::cout << "Run list or range first" << std::endl;
        return std::nullopt;
    }
    std::set<std::size_t> picks;
    for (std::size_t i = first; i < args.size(); i++) {
        for (const std::string & piece : splitOn(args[i], ',')) {
            if (!addRows(piece, session.rows.size(), picks)) {
                return std::nullopt;
            }
        }
    }
    if (picks.empty()) {
        std::cout << "Bad row number: " << args[first] << std::endl;
        return std::nullopt;
    }
    return std::vector<std::size_t>(picks.begin(), picks.end());
}

bool stillThere(Session & session, const std::vector<std::size_t> & picks)
{
    bsoncxx::builder::basic::array ids;
    for (const std::size_t index : picks) {
        ids.append(session.rows[index].id);
    }
    bsoncxx::builder::basic::document filter;
    filter.append(kvp("_id", make_document(kvp("$in", ids.view()))));
    appendScope(filter, session.voyage);
    mongocxx::options::find options;
    options.projection(make_document(kvp("_id", 1), kvp("voyage", 1)));
    std::map<std::string, std::string> found;
    mongocxx::collection               gps = session.db.collection(COLLECTION_GPS);
    for (const DocView & doc : gps.find(filter.view(), options)) {
        found[doc["_id"].get_oid().value.to_string()] = text(doc["voyage"]);
    }
    const std::string where = session.voyage.empty() ? "in the database" : "in voyage " + session.voyage;
    for (const std::size_t index : picks) {
        const Row & row     = session.rows[index];
        const auto  current = found.find(row.id.to_string());
        if (current == found.end()) {
            std::cout << "Row #" << index + 1 << " is no longer " << where << ", run list or range again" << std::endl;
            return false;
        }
        // The selection filter matches each row's voyage, so a row moved since it was listed would match nothing
        if (current->second != row.voyage) {
            std::cout << "Row #" << index + 1 << " is now in voyage " << voyageLabel(current->second)
                      << ", run list or range again" << std::endl;
            return false;
        }
    }
    return true;
}

std::optional<std::vector<std::size_t>> selectRows(Session & session, const Args & args, std::size_t first)
{
    std::optional<std::vector<std::size_t>> picks = parseRows(session, args, first);
    if (picks && !stillThere(session, *picks)) {
        picks.reset();
    }
    return picks;
}

Doc selectionFilter(const Session & session, const std::vector<std::size_t> & picks)
{
    std::vector<Row> rows;
    for (const std::size_t index : picks) {
        rows.push_back(session.rows[index]);
    }
    return rowFilter(rows);
}

Table countTable(Session & session, const Targets & targets)
{
    Table table;
    for (const auto & [name, filter] : targets) {
        mongocxx::collection collection = session.db.collection(name);
        table.push_back({name, std::to_string(collection.count_documents(filter.view()))});
    }
    return table;
}

void printPreview(Session & session, const std::vector<std::size_t> & picks, const Targets & targets)
{
    Table table;
    for (std::size_t i = 0; i < picks.size() && i < PREVIEW_ROWS; i++) {
        table.push_back(rowCells(picks[i], session.rows[picks[i]]));
    }
    printTable(ROW_HEADER, table);
    if (picks.size() > PREVIEW_ROWS) {
        std::cout << "... and " << picks.size() - PREVIEW_ROWS << " more" << std::endl;
    }
    printTable({"collection", "documents"}, countTable(session, targets));
}

void updateAll(Session & session, const Targets & targets, DocView update)
{
    Table table;
    for (const auto & [name, filter] : targets) {
        mongocxx::collection collection = session.db.collection(name);
        const auto           result     = collection.update_many(filter.view(), update);
        table.push_back({name, std::to_string(result ? result->modified_count() : 0)});
    }
    printTable({"collection", "modified"}, table);
}

std::filesystem::path makeBackupFolder()
{
    const std::string           stamp  = formatUtc(std::chrono::system_clock::now(), FOLDER_FORMAT);
    const std::filesystem::path folder = std::filesystem::absolute("backup_" + stamp);
    std::error_code             error;
    if (!std::filesystem::create_directory(folder, error)) {
        throw std::runtime_error("Backup failed, nothing was deleted: cannot create " + folder.string());
    }
    return folder;
}

bsoncxx::array::value backupCollection(
  mongocxx::collection & collection, DocView filter, const std::filesystem::path & file)
{
    std::ofstream                  out(file);
    bsoncxx::builder::basic::array ids;
    for (const DocView & doc : collection.find(filter)) {
        out << bsoncxx::to_json(doc, bsoncxx::ExtendedJsonMode::k_canonical) << "\n";
        ids.append(doc["_id"].get_value());
    }
    out.close();
    if (!out) {
        throw std::runtime_error("Backup failed, nothing was deleted: cannot write " + file.string());
    }
    return ids.extract();
}

std::vector<bsoncxx::array::value> writeBackup(Session & session, const Targets & targets)
{
    const std::filesystem::path        folder = makeBackupFolder();
    std::vector<bsoncxx::array::value> ids;
    for (const auto & [name, filter] : targets) {
        mongocxx::collection collection = session.db.collection(name);
        ids.push_back(backupCollection(collection, filter.view(), folder / (name + ".jsonl")));
    }
    std::cout << "Backup written to " << folder.string() << std::endl;
    return ids;
}

void backupAndDelete(Session & session, const Targets & targets)
{
    const std::vector<bsoncxx::array::value> ids = writeBackup(session, targets);
    Table                                    table;
    for (std::size_t i = 0; i < targets.size(); i++) {
        mongocxx::collection collection = session.db.collection(targets[i].first);
        const auto result = collection.delete_many(make_document(kvp("_id", make_document(kvp("$in", ids[i].view())))));
        table.push_back({targets[i].first, std::to_string(result ? result->deleted_count() : 0)});
    }
    printTable({"collection", "deleted"}, table);
}

void cmdHelp(Session & session, const Args &)
{
    Table table;
    for (const Command & command : commands()) {
        const std::string & usage = session.voyage.empty() ? command.top_usage : command.inside_usage;
        if (!usage.empty()) {
            table.push_back({usage, command.description});
        }
    }
    printTable({"command", "description"}, table);
}

std::vector<std::string> collectionRow(Session & session, const std::string & name)
{
    mongocxx::collection     collection = session.db.collection(name);
    std::vector<std::string> row        = {name, std::to_string(collection.count_documents(make_document()))};
    if (std::find(SENSOR_COLLECTIONS.begin(), SENSOR_COLLECTIONS.end(), name) == SENSOR_COLLECTIONS.end()) {
        return row;
    }
    const Doc tagged   = make_document(kvp("voyage", make_document(kvp("$exists", true))));
    const Doc untagged = make_document(kvp("voyage", make_document(kvp("$exists", false))));
    const Doc bad_time =
      make_document(kvp("timestamp", make_document(kvp("$not", make_document(kvp("$regex", TIMESTAMP_PATTERN))))));
    row.push_back(std::to_string(collection.count_documents(tagged.view())));
    row.push_back(std::to_string(collection.count_documents(untagged.view())));
    row.push_back(std::to_string(collection.count_documents(bad_time.view())));
    return row;
}

void cmdCollections(Session & session, const Args &)
{
    std::vector<std::string> names = session.db.database().list_collection_names();
    std::sort(names.begin(), names.end());
    Table table;
    for (const std::string & name : names) {
        table.push_back(collectionRow(session, name));
    }
    printTable({"collection", "documents", "tagged", "untagged", "bad timestamp"}, table);
}

Doc validTimestamp()
{
    const Doc is_string = make_document(kvp("$eq", make_array(make_document(kvp("$type", "$timestamp")), "string")));
    const Doc as_string = make_document(kvp("$cond", make_array(is_string.view(), "$timestamp", "")));
    const Doc matches =
      make_document(kvp("$regexMatch", make_document(kvp("input", as_string.view()), kvp("regex", TIMESTAMP_PATTERN))));
    return make_document(kvp("$cond", make_array(matches.view(), "$timestamp", bsoncxx::types::b_null{})));
}

std::string firstText(const DocView & doc, const std::string & valid, const std::string & raw)
{
    return decodeTime(bsonString(doc[valid]).value_or(text(doc[raw])));
}

std::map<std::string, VoyageStats> voyageStats(Session & session)
{
    const Doc          valid = validTimestamp();
    mongocxx::pipeline pipeline;
    pipeline.group(make_document(
      kvp("_id", "$voyage"), kvp("messages", make_document(kvp("$sum", 1))),
      kvp("first", make_document(kvp("$min", valid.view()))), kvp("last", make_document(kvp("$max", valid.view()))),
      kvp("raw_first", make_document(kvp("$min", "$timestamp"))),
      kvp("raw_last", make_document(kvp("$max", "$timestamp")))));
    std::map<std::string, VoyageStats> stats;
    mongocxx::collection               gps = session.db.collection(COLLECTION_GPS);
    for (const DocView & doc : gps.aggregate(pipeline)) {
        stats[voyageLabel(text(doc["_id"]))] = {
          fixed(readNumber(doc["messages"]).value_or(0), 0), firstText(doc, "first", "raw_first"),
          firstText(doc, "last", "raw_last")};
    }
    return stats;
}

void cmdVoyages(Session & session, const Args &)
{
    std::map<std::string, VoyageStats> stats  = voyageStats(session);
    const std::string                  active = session.db.getActiveVoyage();
    mongocxx::options::find            options;
    options.sort(make_document(kvp("created_at", -1)));
    Table                table;
    mongocxx::collection voyages = session.db.collection(COLLECTION_VOYAGES);
    for (const DocView & doc : voyages.find(make_document(), options)) {
        const std::string   name = text(doc["name"]);
        const VoyageStats & stat = stats[name];
        table.push_back(
          {name == active ? "*" : "", name, dateText(doc["created_at"]), text(doc["created_by"]), stat.messages,
           stat.first, stat.last});
    }
    const VoyageStats & stat = stats[UNTAGGED];
    table.push_back({"", UNTAGGED, "", "", stat.messages, stat.first, stat.last});
    printTable({"active", "voyage", "created", "by", "messages", "first", "last"}, table);
}

void cmdNewVoyage(Session & session, const Args & args)
{
    if (args.size() > 2) {
        printUsage("newvoyage [name]");
        return;
    }
    const std::string name = args.size() == 2 ? args[1] : session.db.generateVoyageName();
    if (!isValidName(name)) {
        std::cout << NAME_RULE << std::endl;
        return;
    }
    if (voyageExists(session, name)) {
        std::cout << "Voyage " << name << " already exists" << std::endl;
        return;
    }
    if (!confirm("Create voyage " + name + " and make it active for new incoming data?")) {
        return;
    }
    session.db.insertVoyage(name, "cli");
    session.db.activateVoyage(name);
    printActiveVoyage(session);
}

void cmdActivate(Session & session, const Args & args)
{
    if (args.size() != 2) {
        printUsage("activate <voyage>");
        return;
    }
    if (!voyageExists(session, args[1])) {
        printNoVoyage(args[1]);
        return;
    }
    if (isActive(session, args[1])) {
        std::cout << "Voyage " << args[1] << " is already active" << std::endl;
        return;
    }
    if (confirm("Make " + args[1] + " the active voyage for new incoming data?")) {
        session.db.activateVoyage(args[1]);
        printActiveVoyage(session);
    }
}

void renameVoyage(Session & session, const std::string & old_name, const std::string & new_name)
{
    if (!confirm("Rename voyage " + old_name + " to " + new_name + "?")) {
        return;
    }
    session.db.renameActiveVoyage(old_name, new_name);
    mongocxx::collection voyages = session.db.collection(COLLECTION_VOYAGES);
    voyages.update_one(
      make_document(kvp("name", old_name)), make_document(kvp("$set", make_document(kvp("name", new_name)))));
    const Doc update = make_document(kvp("$set", make_document(kvp("voyage", new_name))));
    updateAll(session, sensorTargets(scopeFilter(old_name).view()), update.view());
}

void mergeVoyages(Session & session, const std::string & old_name, const std::string & new_name)
{
    Targets targets = sensorTargets(scopeFilter(old_name).view());
    targets.emplace_back(COLLECTION_VOYAGES, make_document(kvp("name", old_name)));
    std::cout << "Voyage " << new_name << " already exists. This merges the two voyages: every message of " << old_name
              << " moves into " << new_name << " and the voyage " << old_name << " is removed:" << std::endl;
    printTable({"collection", "documents"}, countTable(session, targets));
    if (!confirmWord("merge")) {
        return;
    }
    // Merged messages cannot be told apart afterwards, so keep them as they were (README: undoing a merge)
    writeBackup(session, targets);
    session.db.renameActiveVoyage(old_name, new_name);
    const Doc update = make_document(kvp("$set", make_document(kvp("voyage", new_name))));
    updateAll(session, sensorTargets(scopeFilter(old_name).view()), update.view());
    mongocxx::collection voyages = session.db.collection(COLLECTION_VOYAGES);
    voyages.delete_many(make_document(kvp("name", old_name)));
}

void cmdRename(Session & session, const Args & args)
{
    if (args.size() != 3 || args[1] == args[2]) {
        printUsage("rename <old> <new>");
        return;
    }
    if (!voyageExists(session, args[1])) {
        printNoVoyage(args[1]);
    } else if (!isValidName(args[2])) {
        std::cout << NAME_RULE << std::endl;
    } else if (voyageExists(session, args[2])) {
        mergeVoyages(session, args[1], args[2]);
    } else {
        renameVoyage(session, args[1], args[2]);
    }
}

void cmdEnter(Session & session, const Args & args)
{
    if (args.size() != 2) {
        printUsage("enter <voyage>");
        return;
    }
    if (args[1] != UNTAGGED && !voyageExists(session, args[1])) {
        printNoVoyage(args[1]);
        return;
    }
    session.voyage = args[1];
    session.rows.clear();
}

std::optional<std::int64_t> parseLimit(const Args & args)
{
    if (args.size() == 1) {
        return DEFAULT_LIST_SIZE;
    }
    if (args.size() == 2 && args[1] == "all") {
        return 0;
    }
    const std::size_t limit = args.size() == 2 ? parseNumber(args[1]).value_or(0) : 0;
    if (limit == 0) {
        return std::nullopt;
    }
    return static_cast<std::int64_t>(limit);
}

void cmdList(Session & session, const Args & args)
{
    const std::optional<std::int64_t> limit = parseLimit(args);
    if (!limit) {
        printUsage("list [n|all]");
        return;
    }
    const Doc        filter = stringTimestampFilter(session.voyage);
    std::vector<Row> rows   = findRows(session, filter.view(), -1, *limit);
    std::reverse(rows.begin(), rows.end());
    showRows(session, std::move(rows));
}

void cmdRange(Session & session, const Args & args)
{
    const std::optional<std::string> start = args.size() == 3 ? encodeTime(args[1]) : std::nullopt;
    const std::optional<std::string> end   = args.size() == 3 ? encodeTime(args[2]) : std::nullopt;
    if (!start || !end) {
        printUsage("range <start> <end>");
        std::cout << RANGE_FORMATS << std::endl;
        return;
    }
    bsoncxx::builder::basic::document filter;
    filter.append(kvp("timestamp", make_document(kvp("$gte", *start), kvp("$lt", *end + "~"))));
    appendScope(filter, session.voyage);
    showRows(session, findRows(session, filter.view(), 1, 0));
}

void appendDocuments(Session & session, const std::string & name, DocView filter, Table & table)
{
    mongocxx::collection collection = session.db.collection(name);
    const std::size_t    before     = table.size();
    for (const DocView & doc : collection.find(filter)) {
        for (const bsoncxx::document::element & element : doc) {
            if (keyOf(element) != "timestamp" && keyOf(element) != "voyage") {
                flatten(name, keyOf(element), element, table);
            }
        }
    }
    if (table.size() == before) {
        table.push_back({name, "(no document)", ""});
    }
}

void cmdShow(Session & session, const Args & args)
{
    if (args.size() != 2 || args[1].find_first_of(",-") != std::string::npos) {
        printUsage("show <#>");
        return;
    }
    const std::optional<std::vector<std::size_t>> picks = selectRows(session, args, 1);
    if (!picks) {
        return;
    }
    const Row & row   = session.rows[picks->front()];
    Table       table = {
      {"message", "time", decodeTime(row.timestamp)},
      {"message", "timestamp", row.timestamp},
      {"message", "voyage", voyageLabel(row.voyage)}};
    const Doc filter = rowFilter({row});
    for (const std::string & name : SENSOR_COLLECTIONS) {
        appendDocuments(session, name, filter.view(), table);
    }
    printTable({"collection", "field", "value"}, table);
}

bool acceptTarget(Session & session, const std::string & target, bool & create)
{
    if (target == UNTAGGED) {
        std::cout << "untagged is reserved, use unassign <rows> instead" << std::endl;
        return false;
    }
    if (target == session.voyage) {
        std::cout << "The listed messages are already in voyage " << target << std::endl;
        return false;
    }
    create = !voyageExists(session, target);
    if (create && !isValidName(target)) {
        std::cout << NAME_RULE << std::endl;
        return false;
    }
    return !create || confirm("Voyage " + target + " does not exist. Create it as an inactive voyage?");
}

void cmdAssign(Session & session, const Args & args)
{
    if (args.size() < 3) {
        printUsage("assign <voyage> <rows>");
        return;
    }
    const std::optional<std::vector<std::size_t>> picks  = selectRows(session, args, 2);
    bool                                          create = false;
    if (!picks || !acceptTarget(session, args[1], create)) {
        return;
    }
    const Targets targets = sensorTargets(selectionFilter(session, *picks).view());
    std::cout << "Assign " << picks->size() << " message(s) to voyage " << args[1] << ":" << std::endl;
    printPreview(session, *picks, targets);
    if (!confirm("Are you sure?")) {
        return;
    }
    if (create) {
        session.db.insertVoyage(args[1], "cli");
    }
    updateAll(session, targets, make_document(kvp("$set", make_document(kvp("voyage", args[1])))).view());
    session.rows.clear();
}

void cmdUnassign(Session & session, const Args & args)
{
    if (args.size() < 2) {
        printUsage("unassign <rows>");
        return;
    }
    if (session.voyage == UNTAGGED) {
        std::cout << "The listed messages are already untagged" << std::endl;
        return;
    }
    const std::optional<std::vector<std::size_t>> picks = selectRows(session, args, 1);
    if (!picks) {
        return;
    }
    const Targets targets = sensorTargets(selectionFilter(session, *picks).view());
    std::cout << "Remove the voyage from " << picks->size() << " message(s):" << std::endl;
    printPreview(session, *picks, targets);
    if (confirm("Are you sure?")) {
        updateAll(session, targets, make_document(kvp("$unset", make_document(kvp("voyage", "")))).view());
        session.rows.clear();
    }
}

void cmdDel(Session & session, const Args & args)
{
    if (args.size() < 2) {
        printUsage("del <rows>");
        return;
    }
    const std::optional<std::vector<std::size_t>> picks = selectRows(session, args, 1);
    if (!picks) {
        return;
    }
    const Targets targets = sensorTargets(selectionFilter(session, *picks).view());
    std::cout << "Delete " << picks->size() << " message(s):" << std::endl;
    printPreview(session, *picks, targets);
    if (confirmWord("delete")) {
        backupAndDelete(session, targets);
        session.rows.clear();
    }
}

bool canDelete(Session & session, const std::string & voyage)
{
    if (voyage == UNTAGGED) {
        std::cout << "untagged is not a voyage, use del <rows> to delete untagged messages" << std::endl;
    } else if (!voyageExists(session, voyage)) {
        printNoVoyage(voyage);
    } else if (isActive(session, voyage)) {
        std::cout << "Voyage " << voyage << " is the active voyage and cannot be deleted. Run activate <voyage> or "
                  << "newvoyage first." << std::endl;
    } else {
        return true;
    }
    return false;
}

void cmdDelVoyage(Session & session, const Args & args)
{
    const std::optional<std::string> voyage = targetVoyage(session, args, "delvoyage <voyage>");
    if (!voyage || !canDelete(session, *voyage)) {
        return;
    }
    Targets targets = sensorTargets(scopeFilter(*voyage).view());
    targets.emplace_back(COLLECTION_VOYAGES, make_document(kvp("name", *voyage)));
    std::cout << "Delete voyage " << *voyage << " and every document in it:" << std::endl;
    printTable({"collection", "documents"}, countTable(session, targets));
    if (!confirmWord(*voyage) || !canDelete(session, *voyage)) {
        return;
    }
    backupAndDelete(session, targets);
    if (session.voyage == *voyage) {
        leaveVoyage(session, "Voyage " + *voyage + " was deleted, back at the top level");
    }
}

std::vector<std::string> firstItemCells(const bsoncxx::document::element & array, const CsvSource & source)
{
    std::vector<std::string> cells(source.fields.size());
    if (!array || array.type() != bsoncxx::type::k_array || array.get_array().value.empty()) {
        return cells;
    }
    const bsoncxx::array::element first = *array.get_array().value.begin();
    if (first.type() != bsoncxx::type::k_document) {
        return cells;
    }
    for (std::size_t i = 0; i < cells.size(); i++) {
        cells[i] = csvNumber(first.get_document().value[source.fields[i]]);
    }
    return cells;
}

CsvCells loadSource(Session & session, const CsvSource & source, DocView filter)
{
    CsvCells             cells;
    mongocxx::collection collection = session.db.collection(source.collection);
    for (const DocView & doc : collection.find(filter)) {
        cells.emplace(text(doc["timestamp"]), firstItemCells(doc[source.array], source));
    }
    return cells;
}

std::string csvCell(const std::string & cell)
{
    if (cell.find_first_of(",\"\r\n") == std::string::npos) {
        return cell;
    }
    return "\"" + std::regex_replace(cell, std::regex("\""), "\"\"") + "\"";
}

std::string csvLine(const DocView & gps, const std::vector<CsvCells> & joined)
{
    const std::string        timestamp = text(gps["timestamp"]);
    std::vector<std::string> cells     = {
      decodeTime(timestamp),
      timestamp,
      voyageLabel(text(gps["voyage"])),
      csvNumber(gps["latitude"]),
      csvNumber(gps["longitude"]),
      csvNumber(gps["speed"]),
      csvNumber(gps["heading"])};
    for (std::size_t i = 0; i < CSV_SOURCES.size(); i++) {
        const auto found = joined[i].find(timestamp);
        cells.resize(cells.size() + CSV_SOURCES[i].fields.size());
        if (found != joined[i].end()) {
            std::copy(found->second.begin(), found->second.end(), cells.end() - found->second.size());
        }
    }
    std::string line = csvCell(cells[0]);
    for (std::size_t i = 1; i < cells.size(); i++) {
        line += "," + csvCell(cells[i]);
    }
    return line;
}

bool canExport(Session & session, const std::string & voyage)
{
    if (voyage == UNTAGGED) {
        return true;
    }
    if (!voyageExists(session, voyage)) {
        printNoVoyage(voyage);
        return false;
    }
    if (!isValidName(voyage)) {
        std::cout << "Voyage " << voyage << " cannot be used as a file name, rename it first" << std::endl;
        return false;
    }
    return true;
}

void cmdExport(Session & session, const Args & args)
{
    const std::optional<std::string> voyage = targetVoyage(session, args, "export <voyage>");
    if (!voyage || !canExport(session, *voyage)) {
        return;
    }
    const Doc             filter = stringTimestampFilter(*voyage);
    std::vector<CsvCells> joined;
    for (const CsvSource & source : CSV_SOURCES) {
        joined.push_back(loadSource(session, source, filter.view()));
    }
    mongocxx::options::find options;
    options.sort(make_document(kvp("timestamp", 1), kvp("_id", 1)));
    std::ostringstream   csv;
    std::size_t          count = 0;
    mongocxx::collection gps   = session.db.collection(COLLECTION_GPS);
    csv << CSV_HEADER << "\n";
    for (const DocView & doc : gps.find(filter.view(), options)) {
        csv << csvLine(doc, joined) << "\n";
        count++;
    }
    // Only replace an earlier export once the new one is complete, so a database or disk error never leaves a
    // partial file in its place
    const std::filesystem::path file    = std::filesystem::absolute(*voyage + ".csv");
    const std::filesystem::path partial = std::filesystem::absolute(*voyage + ".csv.part");
    std::ofstream               out(partial);
    out << csv.str();
    out.close();
    if (!out) {
        throw std::runtime_error("Cannot write " + partial.string());
    }
    std::filesystem::rename(partial, file);
    std::cout << "Wrote " << count << " row(s) to " << file.string() << std::endl;
}

void cmdQuit(Session & session, const Args &)
{
    if (session.voyage.empty()) {
        session.running = false;
    } else {
        session.voyage.clear();
        session.rows.clear();
    }
}

void cmdExit(Session & session, const Args &) { session.running = false; }

const std::vector<Command> & commands()
{
    static const std::vector<Command> table = {
      {"help", "help", "help", "Show the commands available here", cmdHelp},
      {"collections", "collections", "", "Every collection with its document counts", cmdCollections},
      {"voyages", "voyages", "", "Every voyage with its message count and time span", cmdVoyages},
      {"newvoyage", "newvoyage [name]", "", "Create a voyage and make it the active one", cmdNewVoyage},
      {"activate", "activate <voyage>", "", "Send new incoming data to an existing voyage", cmdActivate},
      {"rename", "rename <old> <new>", "", "Rename a voyage, or merge it into an existing one", cmdRename},
      {"enter", "enter <voyage>", "", "Work inside one voyage (or untagged) until quit", cmdEnter},
      {"list", "list [n|all]", "list [n|all]", "Latest n messages (default 20), oldest to newest", cmdList},
      {"range", "range <start> <end>", "range <start> <end>", "Messages between two times", cmdRange},
      {"show", "show <#>", "show <#>", "Every field of one listed message", cmdShow},
      {"assign", "assign <voyage> <rows>", "assign <voyage> <rows>", "Move listed messages into a voyage", cmdAssign},
      {"unassign", "unassign <rows>", "unassign <rows>", "Remove the voyage from listed messages", cmdUnassign},
      {"del", "del <rows>", "del <rows>", "Delete listed messages (a backup is written first)", cmdDel},
      {"delvoyage", "delvoyage <voyage>", "delvoyage", "Delete a voyage and all its messages", cmdDelVoyage},
      {"export", "export <voyage>", "export", "Write a voyage to <voyage>.csv", cmdExport},
      {"quit", "quit", "quit", "Leave the voyage, or end the program at the top level", cmdQuit},
      {"exit", "exit", "exit", "End the program", cmdExit}};
    return table;
}

void runCommand(Session & session, const Args & args)
{
    const auto command = std::find_if(
      commands().begin(), commands().end(), [&args](const Command & candidate) { return candidate.name == args[0]; });
    if (command == commands().end()) {
        std::cout << "Unknown command: " << args[0] << " (type help)" << std::endl;
    } else if (!session.voyage.empty() && command->inside_usage.empty()) {
        std::cout << "You are in voyage " << session.voyage << ", type quit to leave it first" << std::endl;
    } else {
        command->run(session, args);
    }
}

void execute(Session & session, const Args & args)
{
    try {
        const bool leaving = args[0] == "quit" || args[0] == "exit";
        if (leaving || !voyageGone(session)) {
            runCommand(session, args);
        }
    } catch (const mongocxx::exception & e) {
        std::cout << "Database error: " << e.what() << std::endl;
    } catch (const std::exception & e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
}

void runLoop(Session & session)
{
    std::string line;
    while (session.running) {
        const std::string scope = session.voyage.empty() ? "" : " [" + session.voyage + "]";
        if (!readLine("\nsailbot" + scope + "> ", line)) {
            std::cout << std::endl;
            break;
        }
        const Args args = splitWords(line);
        if (!args.empty()) {
            execute(session, args);
        }
    }
    std::cout << "bye" << std::endl;
}

int main()
{
    std::cout << "Sailbot DB CLI\nDatabase: " << DB_NAME << "\nMongoDB password: " << std::flush;
    const std::string password = readPassword();
    if (password.empty()) {
        std::cout << "No password entered" << std::endl;
        return 1;
    }
    std::cout << "Connecting to " << MONGODB_ATLAS_HOST << " ... " << std::flush;
    try {
        CliDB db(SailbotDB::atlasConnStr(password));
        if (!db.testConnection()) {
            std::cout << "Connection failed" << std::endl;
            return 1;
        }
        std::cout << "OK" << std::endl;
        Session session{db, "", {}};
        printActiveVoyage(session);
        std::cout << "Type help for commands." << std::endl;
        runLoop(session);
    } catch (const std::exception & e) {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
