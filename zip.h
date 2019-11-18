// File:  zip.h
// Date:  8/4/2016
// Auth:  K. Loux
// Desc:  Zip utilities.

#ifndef ZIP_H_
#define ZIP_H_

// Standard C++ headers
#include <vector>

namespace Utilities
{
	template <typename T1, typename T2>
	std::vector<std::pair<T1, T2> > Zip(const std::vector<T1>& a, const std::vector<T2>& b)
	{
		assert(a.size() == b.size());

		std::vector<std::pair<T1, T2> > z(a.size());
		unsigned int i;
		for (i = 0; i < z.size(); i++)
		{
			z[i].first = a[i];
			z[i].second = b[i];
		}

		return z;
	}

	template <typename T1, typename T2>
	std::vector<std::pair<T1, T2> > Zip(const std::vector<T1>& a, const T2* b)
	{
		std::vector<std::pair<T1, T2> > z(a.size());
		unsigned int i;
		for (i = 0; i < z.size(); i++)
		{
			z[i].first = a[i];
			z[i].second = b[i];
		}

		return z;
	}

	template <typename T1, typename T2>
	std::vector<std::pair<T1, T2> > Zip(const T1* a, const std::vector<T2>& b)
	{
		std::vector<std::pair<T1, T2> > z(b.size());
		unsigned int i;
		for (i = 0; i < z.size(); i++)
		{
			z[i].first = a[i];
			z[i].second = b[i];
		}

		return z;
	}

	template <typename T1, typename T2>
	void Unzip(const std::vector<std::pair<T1, T2> >& z, std::vector<T1>* a, std::vector<T2>* b)
	{
		unsigned int i;

		if (a)
		{
			a->resize(z.size());
			for (i = 0; i < z.size(); i++)
				a->operator[](i) = z[i].first;
		}

		if (b)
		{
			b->resize(z.size());
			for (i = 0; i < z.size(); i++)
				b->operator[](i) = z[i].second;
		}
	}
}// namespace Utilities

#endif// ZIP_H_
