////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include <stdio.h>
#include <istream>
#include <string>
#include <vector>

namespace sbpl {

class CSVParser
{
public:

    bool parseFile(const std::string& path, bool has_header = false);
    bool parseFile(FILE* f, bool has_header = false);

    bool parseStream(std::istream& is, bool has_header = false);

    bool hasHeader() const;

    size_t record_size() const { return m_field_count; }
    const std::vector<std::string>& fields() const { return m_fields; }

private:

    bool m_has_header;
    size_t m_field_count;
    std::vector<std::string> m_fields;

    bool parseRecord(
        std::istream& s,
        std::istream::pos_type& pos,
        std::vector<std::string>& header);

    bool parseField(
        std::istream& s,
        std::istream::pos_type& pos,
        std::string& field);

    bool parseEscaped(
        std::istream& s,
        std::istream::pos_type& pos,
        std::string& text);

    bool parseNonEscaped(
        std::istream& s,
        std::istream::pos_type& pos,
        std::string& text);

    bool parseOptionalHeader(
        std::istream& s,
        std::istream::pos_type& pos,
        std::vector<std::string>& header);

    bool parseHeader(
        std::istream& s,
        std::istream::pos_type& pos,
        std::vector<std::string>& header);

    bool parseTextData(
        std::istream& s,
        std::istream::pos_type& pos,
        std::istream::char_type& c);

    bool parseDoubleQuote(std::istream& s, std::istream::pos_type& pos);

    bool parseDoubleQuote2(std::istream& s, std::istream::pos_type& pos);

    bool parseComma(std::istream& s, std::istream::pos_type& pos);

    bool parseCR(std::istream& s, std::istream::pos_type& pos);

    bool parseLF(std::istream& s, std::istream::pos_type& pos);

    bool parseCRLF(std::istream& s, std::istream::pos_type& pos);

    bool parseName(
        std::istream& s,
        std::istream::pos_type& pos,
        std::string& name);
};

} // namespace sbpl
