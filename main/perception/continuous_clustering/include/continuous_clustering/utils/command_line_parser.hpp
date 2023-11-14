#ifndef CONTINUOUS_CLUSTERING_COMMAND_LINE_PARSER_HPP
#define CONTINUOUS_CLUSTERING_COMMAND_LINE_PARSER_HPP

#include <list>

#include <continuous_clustering/utils/thread_save_queue.hpp>

namespace utils
{
class CommandLineParser
{
  public:
    CommandLineParser(int& argc, char** argv)
    {
        for (int i = 1; i < argc; ++i)
            args.emplace_back(argv[i]);
    }

    const std::string getValueForArgument(const std::string& arg, const std::string& default_if_not_present)
    {
        auto it = std::find(args.begin(), args.end(), arg);
        if (it != args.end())
        {
            it = args.erase(it);
            if (it != args.end())
            {
                std::string value = *it;
                it = args.erase(it);
                return value;
            }
        }
        return default_if_not_present;
    }

    bool argumentExists(const std::string& arg)
    {
        auto it = std::find(args.begin(), args.end(), arg);
        if (it != args.end())
        {
            args.erase(it);
            return true;
        }
        return false;
    }

    const std::list<std::string>& getRemainingArgs()
    {
        return args;
    }

  private:
    std::list<std::string> args;
};

} // namespace utils
#endif