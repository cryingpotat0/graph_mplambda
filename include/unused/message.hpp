#ifndef MPL_MESSAGE_HPP
#define MPL_MESSAGE_HPP

#include <iostream>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>


namespace mpl {
    template <class T>
    class VectorMessage {
        std::vector<T> messages;
        using iterator = typename std::vector<T>::iterator;
        using const_iterator = typename std::vector<T>::const_iterator;
        static inline std::string delimiter = "|";
    public:
        VectorMessage() { };
        explicit VectorMessage(std::vector<T> &messages_) : messages(messages_) {};

        std::string serialize() {
            std::ostringstream oStream;
            for (int i=0; i < messages.size(); ++i) {
                auto msg = messages[i];
                oStream << msg.serialize();
                if (i != messages.size() - 1) {
                    oStream << delimiter;
                }
            }
            return oStream.str();
        }

        static VectorMessage<T> deserialize(std::string &v) {
            std::vector<T> messages;
            std::vector<std::string> results;
            boost::split(results, v, [](char c){return c == '|';});
            for (auto r : results) {
                messages.push_back(std::move(T::deserialize(r)));
            }
            return VectorMessage<T>(messages);
        }

        void insert(T &message) {
            messages.emplace_back(message);
        }

        void clear() {
            messages.clear();
        }

        const std::vector<T>& getMessages() const {
            return messages;
        }
    };

}

#endif //MPL_MESSAGE_HPP
