Summary: Program for converting bitmaps to vector graphics
Name: autotrace
Version: 0.31.1
Release: 1
Url: http://autotrace.sourceforge.net
Source: http://ftp1.sourceforge.net/autotrace/%{name}-%{version}.tar.gz
Copyright: GPL and LGPL
Group: Applications/Graphics
BuildRoot: %{_tmppath}/%{name}-%{version}-root
Prefix: %{_prefix}

%description
AutoTrace is a program for converting bitmaps to vector graphics. The
aim of the AutoTrace project is the development of a freely-available
application similar to CorelTrace or Adobe Streamline. In some
aspects it is already better. Originally being created as a plugin
for the GIMP, AutoTrace is now a standalone program and can be
compiled on any UNIX platform using GCC.


%prep
%setup

%build
%configure --without-magick --without-pstoedit
make

%install
rm -rf %{buildroot}
%makeinstall

%clean
rm -rf %{buildroot}

%files
%defattr(-,root,root)
%doc AUTHORS ChangeLog COPYING* FAQ NEWS README* TODO
%{_bindir}/autotrace
%{_libdir}/libautotrace.*
%{_mandir}/man1/autotrace.1.*
%{_bindir}/autotrace-config
%{_includedir}/autotrace/
%{_datadir}/aclocal/autotrace.m4
%{_libdir}/pkgconfig/autotrace.pc

%changelog
* Thu Jan 23 2003  Masatake YAMATO<jet@gyve.org>
- Import Dag Wieers <dag@wieers.com>'s Updated to 0.31.1

* Wed Oct 23 2002  Masatake YAMATO<jet@gyve.org>
- make disabled magick and pstoedit.

* Tue Jul  9 2002  Masatake YAMATO<jet@gyve.org>
- Supported shared library and bzip'ed manual.

* Sun May 12 2002  Masatake YAMATO<jet@gyve.org>
- Install output.h.

* Tue Apr 16 2002 Masatake YAMATO<jet@gyve.org>
- Added LGPL to Copyright

* Wed Apr  3 2002 Masatake YAMATO<jet@gyve.org>
- autotrace.1 -> autotrace.1.gz

* Thu Mar  7 2002 Masatake YAMATO<jet@gyve.org>
- Change %description.
- Update files.

* Thu Feb 21 2002 Han-Wen Nienhuys <hanwen@cs.uu.nl>
- Initial build.

