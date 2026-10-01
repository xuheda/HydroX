// Copyright (c) 2026 OceanX.
#pragma once
#include <array>
#include <cmath>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

namespace hydrox::bundle_json
{
struct Value
{
    enum Kind
    {
        Object,
        Array,
        String,
        Number,
        Boolean,
        Null
    } kind = Null;
    std::map<std::string, Value> object;
    std::vector<Value> array;
    std::string string;
    double number = 0;
    bool used = false;
};
class Document
{
  public:
    Value root;
    std::string error;
    explicit Document(const std::string &text) : text_(text)
    {
        if (text.size() > 1024 * 1024)
        {
            error = "bundle exceeds 1 MiB";
            return;
        }
        parse(root, 0);
        space();
        if (error.empty() && (pos_ != text_.size() || root.kind != Value::Object))
            error = "expected one JSON object without trailing data";
    }
    Value *at(const std::string &path)
    {
        Value *v = &root;
        v->used = true;
        size_t begin = 0;
        while (begin < path.size())
        {
            const size_t end = path.find('.', begin);
            const auto key = path.substr(begin, end == std::string::npos ? end : end - begin);
            auto it = v->object.find(key);
            if (v->kind != Value::Object || it == v->object.end())
            {
                fail(path + ": required field missing");
                return nullptr;
            }
            v = &it->second;
            v->used = true;
            if (end == std::string::npos)
                break;
            begin = end + 1;
        }
        return v;
    }
    std::string string(const std::string &path)
    {
        auto *v = at(path);
        if (!v || v->kind != Value::String)
        {
            fail(path + ": expected string");
            return {};
        }
        return v->string;
    }
    void read(const std::string &path, double &out)
    {
        auto *v = at(path);
        if (!v || v->kind != Value::Number)
        {
            fail(path + ": expected finite number");
            return;
        }
        out = v->number;
    }
    template <size_t N> void read(const std::string &path, std::array<double, N> &out)
    {
        auto *v = at(path);
        if (!v || v->kind != Value::Array || v->array.size() != N)
        {
            fail(path + ": invalid array size");
            return;
        }
        for (size_t i = 0; i < N; ++i)
        {
            v->array[i].used = true;
            if (v->array[i].kind != Value::Number)
            {
                fail(path + ": expected numeric array");
                return;
            }
            out[i] = v->array[i].number;
        }
    }
    void fail(const std::string &message)
    {
        if (error.empty())
            error = message;
    }
    void reject_unused(const Value &v, const std::string &path = "")
    {
        if (!error.empty())
            return;
        if (!v.used)
        {
            fail(path + ": unknown or inapplicable field");
            return;
        }
        for (const auto &kv : v.object)
            reject_unused(kv.second, path.empty() ? kv.first : path + "." + kv.first);
        for (size_t i = 0; i < v.array.size(); ++i)
            reject_unused(v.array[i], path + "[" + std::to_string(i) + "]");
    }

  private:
    const std::string &text_;
    size_t pos_ = 0;
    void space()
    {
        while (pos_ < text_.size() && (text_[pos_] == ' ' || text_[pos_] == '\r' ||
                                       text_[pos_] == '\n' || text_[pos_] == '\t'))
            ++pos_;
    }
    bool take(char c)
    {
        space();
        if (pos_ < text_.size() && text_[pos_] == c)
        {
            ++pos_;
            return true;
        }
        return false;
    }
    bool quoted(std::string &out)
    {
        if (!take('"'))
            return false;
        while (pos_ < text_.size())
        {
            unsigned char c = text_[pos_++];
            if (c == '"')
                return true;
            if (c < 32)
            {
                fail("control character in JSON string");
                return false;
            }
            if (c == '\\')
            {
                if (pos_ == text_.size())
                    break;
                c = text_[pos_++];
                switch (c)
                {
                case '"':
                case '\\':
                case '/':
                    break;
                case 'b':
                    c = '\b';
                    break;
                case 'f':
                    c = '\f';
                    break;
                case 'n':
                    c = '\n';
                    break;
                case 'r':
                    c = '\r';
                    break;
                case 't':
                    c = '\t';
                    break;
                default:
                    fail("unsupported escape in bundle identifier");
                    return false;
                }
            }
            out.push_back(static_cast<char>(c));
        }
        fail("unterminated JSON string");
        return false;
    }
    void parse(Value &v, int depth)
    {
        if (!error.empty())
            return;
        if (depth > 32)
        {
            fail("JSON nesting exceeds 32");
            return;
        }
        space();
        if (pos_ == text_.size())
        {
            fail("unexpected end of JSON");
            return;
        }
        if (take('{'))
        {
            v.kind = Value::Object;
            if (take('}'))
                return;
            do
            {
                std::string key;
                if (!quoted(key) || !take(':'))
                {
                    fail("expected object key and colon");
                    return;
                }
                if (v.object.count(key))
                {
                    fail("duplicate JSON field: " + key);
                    return;
                }
                parse(v.object[key], depth + 1);
                if (!error.empty())
                    return;
                if (take('}'))
                    return;
            } while (take(','));
            fail("expected object separator");
            return;
        }
        if (take('['))
        {
            v.kind = Value::Array;
            if (take(']'))
                return;
            do
            {
                v.array.emplace_back();
                parse(v.array.back(), depth + 1);
                if (!error.empty())
                    return;
                if (take(']'))
                    return;
            } while (take(','));
            fail("expected array separator");
            return;
        }
        if (text_[pos_] == '"')
        {
            v.kind = Value::String;
            quoted(v.string);
            return;
        }
        for (const char *literal : {"true", "false", "null"})
        {
            const std::string word(literal);
            if (text_.compare(pos_, word.size(), word) == 0)
            {
                v.kind = word == "null" ? Value::Null : Value::Boolean;
                pos_ += word.size();
                return;
            }
        }
        const size_t start = pos_;
        if (text_[pos_] == '-')
            ++pos_;
        auto digit = [&]() {
            return pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9';
        };
        if (!digit())
        {
            fail("expected JSON value");
            return;
        }
        if (text_[pos_] == '0')
            ++pos_;
        else
            while (digit())
                ++pos_;
        if (pos_ < text_.size() && text_[pos_] == '.')
        {
            ++pos_;
            if (!digit())
            {
                fail("invalid fraction");
                return;
            }
            while (digit())
                ++pos_;
        }
        if (pos_ < text_.size() && (text_[pos_] == 'e' || text_[pos_] == 'E'))
        {
            ++pos_;
            if (pos_ < text_.size() && (text_[pos_] == '+' || text_[pos_] == '-'))
                ++pos_;
            if (!digit())
            {
                fail("invalid exponent");
                return;
            }
            while (digit())
                ++pos_;
        }
        const std::string token = text_.substr(start, pos_ - start);
        char *end = nullptr;
        v.number = std::strtod(token.c_str(), &end);
        if (!end || *end || !std::isfinite(v.number))
        {
            fail("non-finite JSON number");
            return;
        }
        v.kind = Value::Number;
    }
};
} // namespace hydrox::bundle_json
