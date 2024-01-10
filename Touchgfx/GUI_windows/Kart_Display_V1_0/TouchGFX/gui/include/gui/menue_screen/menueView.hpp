#ifndef MENUEVIEW_HPP
#define MENUEVIEW_HPP

#include <gui_generated/menue_screen/menueViewBase.hpp>
#include <gui/menue_screen/menuePresenter.hpp>

class menueView : public menueViewBase
{
public:
    menueView();
    virtual ~menueView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // MENUEVIEW_HPP
