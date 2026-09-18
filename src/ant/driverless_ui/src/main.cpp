
/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QApplication>
#include <QDesktopWidget>
#include <QtGui>

#include "driverless.h"
#include <locale.h>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char *argv[])
{
    setlocale(LC_CTYPE,"zh_CN.utf8");
    QApplication a(argc, argv);
    Driverless w(argc, argv);
    w.show();

    return a.exec();
}
