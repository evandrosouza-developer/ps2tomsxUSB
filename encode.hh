/*
 * Copyright (c) 2017, Niklas Gürtler
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
  * This file defines helper functions that are called by the functions from usb_desc_helper.hh.
  * These allow integer data to be encoded in std::array<EncChar,...> instances, and any number of std::array
  * -instances to concatenate, from which the descriptors are ultimately built. EncChar is an alias for uint8_t.
  * All this processing is done "constexpr", i.e. by the compiler, so the result is stored as a constant in Flash
  * can be filed. If the functions from usb_desc_helper.hh are not used, this file can be deleted.
  */

#ifndef ENCODE_HH_
#define ENCODE_HH_

#include <utility>
#include <array>
#include <type_traits>
#include <cstddef>
#include "util.hh"

namespace Util {
	template <typename T>
	using Plain = typename std::remove_cv<typename std::remove_reference<T>::type>::type;

	namespace Helper {
		template <typename T>
		struct ArrSize;

		template <typename T, size_t N>
		struct ArrSize<std::array<T, N>> {
			static constexpr size_t value = N;
		};

		template <typename T, size_t N>
		struct ArrSize<T [N]> {
			static constexpr size_t value = N;
		};
	}

#if (__cplusplus >= 201402L) && (__cpp_fold_expressions >= 201411)

	// Use fold-expression for variadic sum
	template <typename... T>
	usb_always_inline constexpr decltype(auto) staticSum (const T&... x) {
		return (x + ...);
	}
#else
	// Use recursion for variadic sum

	template <typename T>
	usb_always_inline constexpr T staticSum (const T& a) {
		return a;
	}
	template <typename T, typename... Rest>
	usb_always_inline constexpr T staticSum (const T& a, const T& b, const Rest&... rest) {
		return staticSum<T> (a + b, rest...);
	}
#endif

#if (__cplusplus >= 201402L)
	// Use standard static integer list library

	template <size_t... I>
	using IndexSequence = std::index_sequence<I...>;

	template <size_t I>
	using MakeIndexSequence = std::make_index_sequence<I>;
#else
	// Use own implementation for static integer list

	// From https://stackoverflow.com/a/17426611 .

	template <size_t... I>
	struct IndexSequence {};

	template <typename S1, typename S2>
	struct ConcatSequence;

	template <size_t... I1, size_t... I2>
	struct ConcatSequence<IndexSequence<I1...>, IndexSequence<I2...>> {
		using Type = IndexSequence <I1..., (sizeof... (I1) + I2)...>;
	};

	template <size_t N>
	struct GenSeq {
		using Type = typename ConcatSequence<typename GenSeq<N/2>::Type, typename GenSeq<N-N/2>::Type>::Type;
	};
	template <>
	struct GenSeq<0> {
		using Type = IndexSequence<>;
	};
	template <>
	struct GenSeq<1> {
		using Type = IndexSequence<0>;
	};
	template <size_t N>
	using MakeIndexSequence = typename GenSeq<N>::Type;
#endif


	/// Returns the number of elements of the array type specified by "T" (std::array or normal array).
	template <typename T>
	usb_always_inline constexpr size_t arrSize () { return Helper::ArrSize<Plain<T>>::value; }

	namespace Helper {
		template <typename T>
		constexpr const T& cref (T& obj) {
			return obj;
		}

		template <typename T, size_t Length, typename... Seqs>
		struct ConcatArray;

		template <typename T, size_t Length, size_t... Seq1>
		struct ConcatArray<T, Length, IndexSequence<Seq1...>> {
			template <typename Arr1>
			usb_always_inline static constexpr std::array<T, Length> get (Arr1&& arr1) {
				return {{ cref (arr1) [Seq1]... }};
			}
		};

		template <typename T, size_t Length, size_t... Seq1, size_t... Seq2>
		struct ConcatArray<T, Length, IndexSequence<Seq1...>, IndexSequence<Seq2...>> {
			template <typename Arr1, typename Arr2>
			usb_always_inline static constexpr std::array<T, Length> get (Arr1&& arr1, Arr2&& arr2) {
				return {{ cref (arr1) [Seq1]..., cref (arr2) [Seq2]... }};
			}
		};

		template <typename T, size_t Length, size_t... Seq1, size_t... Seq2, size_t... Seq3>
		struct ConcatArray<T, Length, IndexSequence<Seq1...>, IndexSequence<Seq2...>, IndexSequence<Seq3...>> {
			template <typename Arr1, typename Arr2, typename Arr3>
			usb_always_inline static constexpr std::array<T, Length> get (Arr1&& arr1, Arr2&& arr2, Arr3&& arr3) {
				return {{ cref (arr1) [Seq1]..., cref (arr2) [Seq2]..., cref (arr3) [Seq3]... }};
			}
		};

		template <typename T, size_t Length, size_t... Seq1, size_t... Seq2, size_t... Seq3, size_t... Seq4>
		struct ConcatArray<T, Length, IndexSequence<Seq1...>, IndexSequence<Seq2...>, IndexSequence<Seq3...>, IndexSequence<Seq4...>> {
			template <typename Arr1, typename Arr2, typename Arr3, typename Arr4, typename... RestArr>
			usb_always_inline static constexpr std::array<T, Length> get (Arr1&& arr1, Arr2&& arr2, Arr3&& arr3, Arr4&& arr4) {
				return {{ cref (arr1) [Seq1]..., cref (arr2) [Seq2]..., cref (arr3) [Seq3]..., cref (arr4) [Seq4]... }};
			}
		};

		template <typename T, size_t Length, size_t... Seq1, size_t... Seq2, size_t... Seq3, size_t... Seq4, typename... RestSeq>
		struct ConcatArray<T, Length, IndexSequence<Seq1...>, IndexSequence<Seq2...>, IndexSequence<Seq3...>, IndexSequence<Seq4...>, RestSeq...> {
			static constexpr size_t size = sizeof...(Seq1) + sizeof...(Seq2) + sizeof...(Seq3) + sizeof...(Seq4);

			template <typename Arr1, typename Arr2, typename Arr3, typename Arr4, typename... RestArr>
			usb_always_inline static constexpr std::array<T, Length> get (Arr1&& arr1, Arr2&& arr2, Arr3&& arr3, Arr4&& arr4, RestArr&&... rest) {
				return ConcatArray<T, Length, MakeIndexSequence<size>, RestSeq...>
					::get (std::array<T, size> {{ cref (arr1) [Seq1]..., cref (arr2) [Seq2]..., cref (arr3) [Seq3]..., cref (arr4) [Seq4]... }}, std::forward<RestArr> (rest)...);
			}

		};
	}

	using EncChar = uint8_t;

	/**
	* Concatenates the sequence of arrays specified by parameter into an array. The element type must be specified in "T".
	* A 4x fold is applied to concatenate 4 arrays at a time.
	*/
	template <typename T, typename... Arrays>
	usb_always_inline constexpr std::array<T, staticSum<size_t> (arrSize<Arrays> ()...)> concatArrays (Arrays&&... arrays) {
		return Helper::ConcatArray<T, staticSum<size_t> (arrSize<Arrays> ()...), MakeIndexSequence<arrSize<Arrays> ()>...>::get (std::forward<Arrays> (arrays)...);
	}

	namespace Helper {
		template <typename T, size_t... I>
		usb_always_inline constexpr std::array<EncChar, sizeof(T)> encodeUInt (T&& val, IndexSequence<I...>) {
			return {{ static_cast<EncChar> ((val >> (8 * I)) & 0xFF) ... }};
		}

		template <typename T, size_t N, size_t... I>
		usb_always_inline constexpr std::array<EncChar, sizeof...(I)> encodeString (const T (&string) [N], IndexSequence<I...>) {
			return {{ static_cast<EncChar> (static_cast<typename std::make_unsigned<T>::type> ( string [I/sizeof(T)] ) >> ((I%sizeof(T))*8))... }};
		}
	}

	/// Encodes an unsigned integer into a std::array<EncChar,...>.
	template <typename T>
	usb_always_inline constexpr typename std::enable_if<std::is_unsigned<Plain<T>>::value, std::array<EncChar, sizeof(T)>>::type encode (T&& val) {
		return Helper::encodeUInt (std::forward<T> (val), MakeIndexSequence<sizeof(T)> {});
	}
	/// Encodes a signed integer into a std::array<EncChar,...>.
	template <typename T>
	usb_always_inline constexpr typename std::enable_if<std::is_signed<Plain<T>>::value, std::array<EncChar, sizeof(T)>>::type encode (T&& val) {
		return Helper::encodeUInt (static_cast<typename std::make_unsigned<Plain<T>>::type> (val), MakeIndexSequence<sizeof(T)> {});
	}
	/// Encodes an enum value into a std::array<EncChar,...> using the underlying type.
	template <typename T>
	usb_always_inline constexpr decltype(encode(std::declval<typename std::underlying_type<typename std::enable_if<std::is_enum<Plain<T>>::value, Plain<T>>::type>::type>())) encode (T&& val) {
		return encode (static_cast<typename std::underlying_type<Plain<T>>::type> (std::forward<T> (val)));
	}

	/**
	* Converts a string literal (char, charXX_t) to a std::array<EncChar,...>, where the terminating 0 character
	* is truncated. char16_t/char32_t strings (typically UTF-16/32) are encoded as little endian.
	*/	
	template <typename T, size_t N>
	usb_always_inline constexpr std::array<EncChar, sizeof(T)*(N-1)> encodeString (const T (&string) [N]) {
		return Helper::encodeString (string, MakeIndexSequence<sizeof(T)*(N-1)> {});
	}

	namespace Helper {
		template <typename T, size_t N, typename ArrT, size_t... I>
		usb_always_inline constexpr std::array<EncChar, N * arrSize<decltype(encode (std::declval<T> ()))> ()> encodeArray (ArrT&& arr, IndexSequence<I...>) {
			return concatArrays<EncChar> (encode (std::forward<ArrT> (arr) [I])...);
		}
	}
	/**
	* Converts an array of any type to a std::array<EncChar,...>, dividing the elements individually using the appropriate
	* encode() function to be converted.
	*/
	template <typename T, size_t N>
	usb_always_inline constexpr std::array<EncChar, N * arrSize<decltype(encode (std::declval<T> ()))> ()> encode (std::array<T, N>&& arr) {
		return Helper::encodeArray<T, N> (std::move (arr), MakeIndexSequence<N> {});
	}
	/**
	* Converts an array of any type to a std::array<EncChar,...>, dividing the elements individually using the appropriate
	* encode() function to be converted.
	*/
	template <typename T, size_t N>
	usb_always_inline constexpr std::array<EncChar, N * arrSize<decltype(encode (std::declval<T> ()))> ()> encode (const std::array<T, N>& arr) {
		return Helper::encodeArray<T, N> (arr, MakeIndexSequence<N> {});
	}
	/**
	* Converts an array of any type to a std::array<EncChar,...>, dividing the elements individually using the appropriate
	* encode() function to be converted.
	*/
	template <typename T, size_t N>
	usb_always_inline constexpr std::array<EncChar, N * arrSize<decltype(encode (std::declval<T> ()))> ()> encode (T (&&arr) [N]) {
		return Helper::encodeArray<T, N> (std::move (arr), MakeIndexSequence<N> {});
	}
	/**
	* Converts an array of any type to a std::array<EncChar,...>, dividing the elements individually using the appropriate
	* encode() function to be converted.
	*/
	template <typename T, size_t N>
	usb_always_inline constexpr std::array<EncChar, N * arrSize<decltype(encode (std::declval<T> ()))> ()> encode (const T (&arr) [N]) {
		return Helper::encodeArray<T, N> (arr, MakeIndexSequence<N> {});
	}
	/// Returns the parameter directly.
	template <size_t N> usb_always_inline constexpr std::array<EncChar, N> encode (std::array<EncChar, N>&& arr) { return arr;	}
	/// Returns the parameter directly.
	template <size_t N> usb_always_inline constexpr std::array<EncChar, N> encode (const std::array<EncChar, N>& arr) { return arr; }

	/// Encodes each parameter using the appropriate encode() function, and returns the concatenated results.
	template <typename... T>
	usb_always_inline constexpr std::array<EncChar, staticSum<size_t> (arrSize<decltype(encode (std::declval<T> ()))> ()...)> encodeMulti (T&&... args) {
		return concatArrays<EncChar> (encode (args)...);
	}
}


#endif /* ENCODE_HH_ */
